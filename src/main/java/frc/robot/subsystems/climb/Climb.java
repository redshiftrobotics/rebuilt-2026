package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;

import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Climb. */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /** Creates a new Climb subsystem compatible with the given robot type. */
  public Climb create(RobotType robotType) {
    switch (robotType) {
      case SIM_BOT:
        return new Climb(new ClimbIOSim());
      default:
        return new Climb(new ClimbIO() {});
    }
  }

  private Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  /** Run the motor at a given speed */
  public void start() {
    io.setSpeed(ClimbConstants.SPEED);
  }

  /** Stop the motor */
  public void stop() {
    io.setSpeed(0.0);
  }

  public double getRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
