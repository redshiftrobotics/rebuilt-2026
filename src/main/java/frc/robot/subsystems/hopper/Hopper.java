package frc.robot.subsystems.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.hopper.HopperConstants.RunMode;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final HopperMotorIO feederIO;
  private final HopperMotorIO lifterIO;

  /* Loggable inputs */
  private final HopperMotorIOInputsAutoLogged feederInputs = new HopperMotorIOInputsAutoLogged();
  private final HopperMotorIOInputsAutoLogged lifterInputs = new HopperMotorIOInputsAutoLogged();

  private final Alert feederDisconnectedAlert =
      new Alert("Hardware error detected on feeder.", AlertType.kError);
  private final Alert lifterMotorDisconnectedAlert =
      new Alert("Hardware error detected lifter.", AlertType.kError);

  private final TunablePID feederPidConfig =
      new TunablePID(getName() + "/FeederPID", HopperConstants.FEEDER_PID);
  private final TunablePID lifterPidConfig =
      new TunablePID(getName() + "/LifterPID", HopperConstants.LIFTER_PID);

  /* Mechanism visualization */
  private final HopperVisualizer measuredVisualizer;
  private final HopperVisualizer setpointVisualizer;

  /* Run mode storage */
  private HopperConstants.RunMode runMode = RunMode.STOPPED;

  private double setpointFeederRPM;
  private double setpointLifterRPM;

  public Hopper(HopperMotorIO feederIO, HopperMotorIO lifterIO) {
    this.feederIO = feederIO;
    this.lifterIO = lifterIO;

    feederIO.setPID(HopperConstants.FEEDER_PID);
    lifterIO.setPID(HopperConstants.LIFTER_PID);

    measuredVisualizer = new HopperVisualizer(getName() + "/Visuization/Measured", Color.kRed);
    setpointVisualizer = new HopperVisualizer(getName() + "/Visuization/Setpoint", Color.kBlue);
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);
    lifterIO.updateInputs(lifterInputs);

    Logger.processInputs(getName() + "/Feeder", feederInputs);
    Logger.processInputs(getName() + "/Lifter", lifterInputs);

    feederPidConfig.ifChanged(hashCode(), feederIO::setPID);
    lifterPidConfig.ifChanged(hashCode(), lifterIO::setPID);

    measuredVisualizer.update(getfeederVelocityRPM(), getlifterVelocityRPM());
    setpointVisualizer.update(setpointFeederRPM, setpointLifterRPM);

    feederDisconnectedAlert.set(!feederInputs.motorConnected);
    lifterMotorDisconnectedAlert.set(!lifterInputs.motorConnected);
  }

  private void runfeederAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    feederIO.setVelocity(velocityRadPerSec, 0);

    setpointFeederRPM = velocityRPM;
    Logger.recordOutput(getName() + "/Feeder/SetpointRPM", velocityRPM);
  }

  private void runlifterAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    lifterIO.setVelocity(velocityRadPerSec, 0);

    setpointLifterRPM = velocityRPM;
    Logger.recordOutput(getName() + "/Lifter/SetpointRPM", velocityRPM);
  }

  public void stopfeeder() {
    runfeederAtVelocity(0);
    feederIO.stop();
  }

  public void stoplifter() {
    runlifterAtVelocity(0);
    lifterIO.stop();
  }

  public void stopAll() {
    runMode = HopperConstants.RunMode.STOPPED;
    stopfeeder();
    stoplifter();
  }

  public void runInMode(HopperConstants.RunMode mode) {
    runMode = mode;
    if (mode == HopperConstants.RunMode.STOPPED) {
      stopAll();
      return;
    }
    runfeederAtVelocity(mode.feederVelocityRadPerSec);
    runlifterAtVelocity(mode.lifterVelocityRadPerSec);
  }

  public HopperConstants.RunMode getCurrentRunMode() {
    return runMode;
  }

  @AutoLogOutput
  public double getfeederVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(feederInputs.velocityRadPerSec);
  }

  public double getFeederCharacterizationVelocity() {
    return feederInputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public double getlifterVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(lifterInputs.velocityRadPerSec);
  }

  public double getLifterCharacterizationVelocity() {
    return lifterInputs.velocityRadPerSec;
  }

  public static Hopper create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Hopper(
            new HopperMotorIOSparkMax(
                HopperConstants.FEEDER_CAN_ID,
                HopperConstants.FEEDER_GEAR_RATIO,
                HopperConstants.FEEDER_INVERTED),
            new HopperMotorIOSparkMax(
                HopperConstants.LIFTER_CAN_ID,
                HopperConstants.LIFTER_GEAR_RATIO,
                HopperConstants.LIFTER_INVERTED));

      case SIM_BOT:
        return new Hopper(
            new HopperMotorIOSim(HopperConstants.FEEDER_GEAR_RATIO),
            new HopperMotorIOSim(HopperConstants.LIFTER_GEAR_RATIO));

      default:
        return new Hopper(new HopperMotorIO() {}, new HopperMotorIO() {});
    }
  }
}
