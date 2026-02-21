package frc.robot.subsystems.hopper;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.common.motorio.MotorIO;
import frc.robot.subsystems.common.motorio.MotorIOInputsAutoLogged;
import frc.robot.subsystems.common.motorio.MotorIOSim;
import frc.robot.subsystems.common.motorio.MotorIOSparkMax;
import frc.robot.subsystems.hopper.HopperConstants.RunMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final MotorIO feeder;
  private final HopperMotorIO lifter;

  /* Loggable inputs */
  private final MotorIOInputsAutoLogged feederInputs = new MotorIOInputsAutoLogged();
  private final HopperMotorIOInputsAutoLogged lifterInputs = new HopperMotorIOInputsAutoLogged();

  /* Run mode storage */
  private HopperConstants.RunMode runMode = RunMode.STOPPED;

  /* Mechanism visualization */
  private final HopperVisualizer visualizer;

  public Hopper(MotorIO feederIO, HopperMotorIO lifterIO) {
    // Set IO layers
    feeder = feederIO;
    lifter = lifterIO;

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

  private void runlifterAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    lifter.setVelocity(velocityRadPerSec, HopperConstants.LIFTER_FF_VOLTS);

    Logger.recordOutput("Hopper/lifter/SetpointRPM", velocityRPM);
  }

  public void stopfeeder() {
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
    feeder.setVoltage(mode.feederVolts);
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
            new MotorIOSim(HopperConstants.FEEDER_GEAR_RATIO),
            new HopperMotorIOSim(HopperConstants.LIFTER_GEAR_RATIO));

      default:
        return new Hopper(new MotorIOSparkMax(HopperConstants.FEEDER_CAN_ID, HopperConstants.FEEDER_INVERTED, HopperConstants.FEEDER_GEAR_RATIO), new HopperMotorIO() {});
    }
  }
}
