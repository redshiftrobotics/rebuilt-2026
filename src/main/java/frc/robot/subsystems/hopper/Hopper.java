package frc.robot.subsystems.hopper;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.hopper.HopperConstants.RunMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

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

  /* Run mode storage */
  private HopperConstants.RunMode runMode = RunMode.STOPPED;

  /* Mechanism visualization */
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d bubblerRoot;
  private final LoggedMechanismRoot2d feederRoot;
  private final LoggedMechanismLigament2d bubblerMech;
  private final LoggedMechanismLigament2d feederMech;

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

    // Set up mechanism
    mechanism = new LoggedMechanism2d(256, 256);
    bubblerRoot = mechanism.getRoot("Bubbler", 128, 64);
    bubblerMech = new LoggedMechanismLigament2d("Bubbler", 25, 0, 5, new Color8Bit(Color.kViolet));
    bubblerRoot.append(bubblerMech);
    feederRoot = mechanism.getRoot("Feeder", 128, 192);
    feederMech = new LoggedMechanismLigament2d("Feeder", 25, 0, 5, new Color8Bit(Color.kGoldenrod));
    feederRoot.append(feederMech);
  }

  @Override
  public void periodic() {
    // Update and log inputs
    bubbler.updateInputs(bubblerInputs);
    feeder.updateInputs(feederInputs);
    Logger.processInputs("Hopper/Bubbler", bubblerInputs);
    Logger.processInputs("Hopper/Feeder", feederInputs);

    // Update and log mechanisms
    bubblerMech.setAngle(Units.radiansToDegrees(bubblerInputs.positionRad));
    feederMech.setAngle(Units.radiansToDegrees(feederInputs.positionRad));
    Logger.recordOutput("Hopper/Visualization", mechanism);
  }

  private void runBubblerAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    bubbler.setVelocity(velocityRadPerSec, bubblerFF.calculate(velocityRadPerSec));

    Logger.recordOutput("Hopper/Bubbler/SetpointRPM", velocityRPM);
  }

  private void runFeederAtVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    feeder.setVelocity(velocityRadPerSec, feederFF.calculate(velocityRadPerSec));

    Logger.recordOutput("Hopper/Feeder/SetpointRPM", velocityRPM);
  }

  public void stopBubbler() {
    runBubblerAtVelocity(0);
    bubbler.stop();
  }

  public void stopFeeder() {
    runFeederAtVelocity(0);
    feeder.stop();
  }

  public void stopAll() {
    runMode = HopperConstants.RunMode.STOPPED;
    stopBubbler();
    stopFeeder();
  }

  public void runInMode(HopperConstants.RunMode mode) {
    runMode = mode;
    if (mode == HopperConstants.RunMode.STOPPED) {
      stopAll();
      return;
    }
    runBubblerAtVelocity(mode.bubblerVelocityRadPerSec);
    runFeederAtVelocity(mode.feederVelocityRadPerSec);
  }

  public HopperConstants.RunMode getCurrentRunMode() {
    return runMode;
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

public static Hopper create(RobotType robotType) {
    switch (robotType) {
      case SIM_BOT:
        return 
            new Hopper(
                new HopperMotorIOSim(HopperConstants.BUBBLER_GEAR_RATIO),
                new HopperMotorIOSim(HopperConstants.FEEDER_GEAR_RATIO));
    
      default:
   return   new Hopper(new HopperMotorIO() {}, new HopperMotorIO() {});
   
    }
}
}
