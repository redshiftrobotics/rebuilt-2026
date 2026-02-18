package frc.robot.subsystems.hopper;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.hopper.HopperConstants.RunMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final HopperMotorIO feeder;
  private final HopperMotorIO lifter;

  /* Loggable inputs */
  private final HopperMotorIOInputsAutoLogged feederInputs = new HopperMotorIOInputsAutoLogged();
  private final HopperMotorIOInputsAutoLogged lifterInputs = new HopperMotorIOInputsAutoLogged();

  /* Feedforward models */
  private final SimpleMotorFeedforward feederFF;
  private final SimpleMotorFeedforward lifterFF;

  /* Run mode storage */
  private HopperConstants.RunMode runMode = RunMode.STOPPED;

  /* Mechanism visualization */
  private final HopperVisualizer visualizer;

  public Hopper(HopperMotorIO feederIO, HopperMotorIO lifterIO) {
    // Set IO layers
    feeder = feederIO;
    lifter = lifterIO;

    // Configure feedforward models
    feederFF =
        new SimpleMotorFeedforward(
            HopperConstants.FEEDER_FF.kS(),
            HopperConstants.FEEDER_FF.kV(),
            HopperConstants.FEEDER_FF.kA());
    lifterFF =
        new SimpleMotorFeedforward(
            HopperConstants.LIFTER_FF.kS(),
            HopperConstants.LIFTER_FF.kV(),
            HopperConstants.LIFTER_FF.kA());

    // Apply PID constants
    feeder.configurePID(
        HopperConstants.FEEDER_PID.kP(),
        HopperConstants.FEEDER_PID.kI(),
        HopperConstants.FEEDER_PID.kD());
    lifter.configurePID(
        HopperConstants.LIFTER_PID.kP(),
        HopperConstants.LIFTER_PID.kI(),
        HopperConstants.LIFTER_PID.kD());

    visualizer = new HopperVisualizer(Color.kGray, Color.kGreen);
  }

  @Override
  public void periodic() {
    // Update and log inputs
    feeder.updateInputs(feederInputs);
    lifter.updateInputs(lifterInputs);
    Logger.processInputs("Hopper/feeder", feederInputs);
    Logger.processInputs("Hopper/lifter", lifterInputs);

    // Update and log mechanisms
    visualizer.think(feederInputs, lifterInputs);
  }

  private void runfeederAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    feeder.setVelocity(velocityRadPerSec, feederFF.calculate(velocityRadPerSec));

    Logger.recordOutput("Hopper/feeder/SetpointRPM", velocityRPM);
  }

  private void runlifterAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    lifter.setVelocity(velocityRadPerSec, lifterFF.calculate(velocityRadPerSec));

    Logger.recordOutput("Hopper/lifter/SetpointRPM", velocityRPM);
  }

  public void stopfeeder() {
    runfeederAtVelocity(0);
    feeder.stop();
  }

  public void stoplifter() {
    runlifterAtVelocity(0);
    lifter.stop();
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
      case SIM_BOT:
        return new Hopper(
            new HopperMotorIOSim(HopperConstants.FEEDER_GEAR_RATIO),
            new HopperMotorIOSim(HopperConstants.LIFTER_GEAR_RATIO));

      default:
        return new Hopper(new HopperMotorIO() {}, new HopperMotorIO() {});
    }
  }
}
