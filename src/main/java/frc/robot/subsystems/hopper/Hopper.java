package frc.robot.subsystems.hopper;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.hopper.HopperConstants.HopperRunMode;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final HopperMotorIO feederIO;
  private final HopperMotorIO lifterIO;

  /* Loggable inputs */
  private final HopperMotorIOInputsAutoLogged feederInputs = new HopperMotorIOInputsAutoLogged();
  private final HopperMotorIOInputsAutoLogged lifterInputs = new HopperMotorIOInputsAutoLogged();

  private final Alert feederDisconnectedAlert =
      new Alert("Feeder motor disconnected, ", AlertType.kError);
  private final Alert lifterMotorDisconnectedAlert =
      new Alert("Lifter motor disconnected", AlertType.kError);

  /* Mechanism visualization */
  private final HopperVisualizer measuredVisualizer;
  private final HopperVisualizer setpointVisualizer;

  /* Run mode storage */
  private HopperConstants.HopperRunMode runMode = HopperRunMode.STOPPED;

  public Hopper(HopperMotorIO feederIO, HopperMotorIO lifterIO) {
    this.feederIO = feederIO;
    this.lifterIO = lifterIO;

    measuredVisualizer = new HopperVisualizer(getName() + "/Visualization/Measured", Color.kRed, 0);
    setpointVisualizer =
        new HopperVisualizer(getName() + "/Visualization/Setpoint", Color.kBlue, 0.01);

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

    feederDisconnectedAlert.set(!feederInputs.motorConnected);
    lifterMotorDisconnectedAlert.set(!lifterInputs.motorConnected);

    measuredVisualizer.update(feederInputs.velocityRadPerSec, lifterInputs.velocityRadPerSec);
  }

  public Command runModeCommand(HopperConstants.HopperRunMode mode) {
    return startEnd(() -> setMode(mode), this::stopAll);
  }

  public void stopAll() {
    setMode(HopperConstants.HopperRunMode.STOPPED);
  }

  public void setMode(HopperRunMode mode) {

    setpointVisualizer.update(
        mode.feederDutyCycle * HopperConstants.MAX_FEEDER_SPEED,
        mode.lifterDutyCycle * HopperConstants.MAX_LIFTER_SPEED);

    if (mode == HopperRunMode.STOPPED) {
      lifterIO.stop();
      feederIO.stop();
    } else {
      feederIO.setDutyCycle(mode.feederDutyCycle);
      lifterIO.setDutyCycle(mode.lifterDutyCycle);
    }

    runMode = mode;
  }

  public HopperRunMode getCurrentRunMode() {
    return runMode;
  }

  public static Hopper create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Hopper(
            new HopperMotorIOSparkMax(HopperConstants.FEEDER_CONSTANTS),
            new HopperMotorIOSparkMax(HopperConstants.LIFTER_CONSTANTS));

      case SIM_BOT:
        return new Hopper(
            new HopperMotorIOSim(
                HopperConstants.LIFTER_MOTOR, HopperConstants.FEEDER_CONSTANTS, 0.1),
            new HopperMotorIOSim(
                HopperConstants.FEEDER_MOTOR, HopperConstants.LIFTER_CONSTANTS, 0.1));

      default:
        return new Hopper(new HopperMotorIO() {}, new HopperMotorIO() {});
    }
  }
}
