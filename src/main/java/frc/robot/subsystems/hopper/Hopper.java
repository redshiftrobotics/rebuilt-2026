package frc.robot.subsystems.hopper;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final HopperMotorIO bubbler;
  private final HopperMotorIO feeder;

  /* Loggable inputs */
  private final HopperMotorIOInputsAutoLogged bubblerInputs = new HopperMotorIOInputsAutoLogged();
  private final HopperMotorIOInputsAutoLogged feederInputs = new HopperMotorIOInputsAutoLogged();

  /* Feedforward models */
  private final SimpleMotorFeedforward bubblerFF;
  private final SimpleMotorFeedforward feederFF;

  public Hopper(HopperMotorIO bubblerIO, HopperMotorIO feederIO) {
    // Set IO layers
    bubbler = bubblerIO;
    feeder = feederIO;

    // Configure feedforward models
    bubblerFF =
        new SimpleMotorFeedforward(
            HopperConstants.BUBBLER_FF.kS(),
            HopperConstants.BUBBLER_FF.kV(),
            HopperConstants.BUBBLER_FF.kA());
    feederFF =
        new SimpleMotorFeedforward(
            HopperConstants.FEEDER_FF.kS(),
            HopperConstants.FEEDER_FF.kV(),
            HopperConstants.FEEDER_FF.kA());

    // Apply PID constants
    bubbler.configurePID(
        HopperConstants.BUBBLER_PID.kP(),
        HopperConstants.BUBBLER_PID.kI(),
        HopperConstants.BUBBLER_PID.kD());
    feeder.configurePID(
        HopperConstants.FEEDER_PID.kP(),
        HopperConstants.FEEDER_PID.kI(),
        HopperConstants.FEEDER_PID.kD());
  }

  @Override
  public void periodic() {
    // Update and log inputs
    bubbler.updateInputs(bubblerInputs);
    feeder.updateInputs(feederInputs);
    Logger.processInputs("Hopper/Bubbler", bubblerInputs);
    Logger.processInputs("Hopper/Feeder", feederInputs);
  }

  public void runBubblerAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    bubbler.setVelocity(velocityRadPerSec, bubblerFF.calculate(velocityRadPerSec));

    Logger.recordOutput("Hopper/Bubbler/SetpointRPM", velocityRPM);
  }

  public void runFeederAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    feeder.setVelocity(velocityRadPerSec, feederFF.calculate(velocityRadPerSec));

    Logger.recordOutput("Hopper/Feeder/SetpointRPM", velocityRPM);
  }

  public void stopBubbler() {
    runBubblerAtVelocity(0);
    bubbler.stop();
  }

  public void stopFeeder() {
    runBubblerAtVelocity(0);
    feeder.stop();
  }

  public void stopAll() {
    stopBubbler();
    stopFeeder();
  }

  @AutoLogOutput
  public double getBubblerVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(bubblerInputs.velocityRadPerSec);
  }

  public double getBubblerCharacterizationVelocity() {
    return bubblerInputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public double getFeederVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(feederInputs.velocityRadPerSec);
  }

  public double getFeederCharacterizationVelocity() {
    return feederInputs.velocityRadPerSec;
  }
}
