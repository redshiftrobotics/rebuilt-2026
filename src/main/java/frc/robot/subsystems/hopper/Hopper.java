package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.common.velocityMotor.MotorIO;
import frc.robot.subsystems.common.velocityMotor.MotorIOInputsAutoLogged;
import frc.robot.subsystems.common.velocityMotor.MotorIOSim;
import frc.robot.subsystems.common.velocityMotor.MotorIOSparkMax;
import frc.robot.subsystems.hopper.HopperConstants.HopperRunMode;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final MotorIO feederIO;
  private final MotorIO lifterIO;

  /* Loggable inputs */
  private final MotorIOInputsAutoLogged feederInputs = new MotorIOInputsAutoLogged();
  private final MotorIOInputsAutoLogged lifterInputs = new MotorIOInputsAutoLogged();

  private final Alert feederDisconnectedAlert =
      new Alert("Feeder motor disconnected, ", AlertType.kError);
  private final Alert lifterMotorDisconnectedAlert =
      new Alert("Lifter motor disconnected", AlertType.kError);

  private final TunablePID lifterPidConfig =
      new TunablePID(getName() + "/LifterPID", HopperConstants.LIFTER_PID);

  /* Mechanism visualization */
  private final HopperVisualizer measuredVisualizer;
  private final HopperVisualizer setpointVisualizer;

  /* Run mode storage */
  private HopperConstants.HopperRunMode runMode = HopperRunMode.STOPPED;

  public Hopper(MotorIO feederIO, MotorIO lifterIO) {
    this.feederIO = feederIO;
    this.lifterIO = lifterIO;

    lifterIO.setPID(HopperConstants.LIFTER_PID);

    measuredVisualizer = new HopperVisualizer(getName() + "/Visuization/Measured", Color.kRed, 0);
    setpointVisualizer =
        new HopperVisualizer(getName() + "/Visuization/Setpoint", Color.kBlue, 0.01);

    stopAll();

    SmartDashboard.putData(
        "Hopper State",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addStringProperty("Run Mode", () -> runMode.toString(), null);
            builder.addDoubleProperty(
                "Feeder Velocity (rad per sec)", () -> feederInputs.velocityRadPerSec, null);
            builder.addDoubleProperty(
                "Lifter Velocity (rad per sec)", () -> lifterInputs.velocityRadPerSec, null);
            builder.addDoubleProperty(
                "Feeder Dutycycle", () -> feederInputs.appliedDutycycle, null);
            builder.addDoubleProperty(
                "Lifter Dutycycle", () -> lifterInputs.appliedDutycycle, null);
          }
        });
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);
    lifterIO.updateInputs(lifterInputs);

    Logger.processInputs(getName() + "/Feeder", feederInputs);
    Logger.processInputs(getName() + "/Lifter", lifterInputs);

    lifterPidConfig.ifChanged(hashCode(), lifterIO::setPID);

    feederDisconnectedAlert.set(!feederInputs.motorConnected);
    lifterMotorDisconnectedAlert.set(!lifterInputs.motorConnected);

    measuredVisualizer.update(feederInputs.velocityRadPerSec, lifterInputs.velocityRadPerSec);
  }

  public Command runModeCommand(HopperConstants.HopperRunMode mode) {
    return startEnd(() -> setMode(mode), this::stopAll);
  }

  public void stopAll() {
    runMode = HopperConstants.HopperRunMode.STOPPED;
    setpointVisualizer.update(0.0, 0.0);
    feederIO.stop();
    lifterIO.stop();
  }

  public void setMode(HopperConstants.HopperRunMode mode) {
    runMode = mode;
    if (mode == HopperConstants.HopperRunMode.STOPPED) {
      stopAll();
      return;
    }
    feederIO.setDutyCycle(mode.feederDutyCycle);
    lifterIO.setVelocity(mode.lifterVelocityRadPerSec);

    setpointVisualizer.update(
        mode.feederDutyCycle * HopperConstants.MAX_FEEDER_SPEED, mode.lifterVelocityRadPerSec);
  }

  public HopperConstants.HopperRunMode getCurrentRunMode() {
    return runMode;
  }

  public static Hopper create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Hopper(
            new MotorIOSparkMax(HopperConstants.FEEDER_CONSTANTS),
            new MotorIOSparkMax(HopperConstants.LIFTER_CONSTANTS));

      case SIM_BOT:
        return new Hopper(
            new MotorIOSim(DCMotor.getNEO(1), HopperConstants.FEEDER_CONSTANTS, 0.1),
            new MotorIOSim(DCMotor.getNEO(1), HopperConstants.LIFTER_CONSTANTS, 0.1));

      default:
        return new Hopper(new MotorIO() {}, new MotorIO() {});
    }
  }
}
