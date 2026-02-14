package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import org.littletonrobotics.junction.Logger;

/**
 * The subsystem that controls the climber mechanism on the robot. Adapted from: {@link
 * https://github.com/redshiftrobotics/crescendo-2024/tree/main/src/main/java/frc/robot/subsystems/hang}
 *
 * @author Aceius E.
 */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /**
   * Creates a new Climb subsystem compatible with the given robot type.
   *
   * @param robotType The robot type to configure for.
   * @return An instance of the Climb subsystem with appropriate IO layers for the given robot.
   */
  public Climb create(RobotType robotType) {
    switch (robotType) {
      case SIM_BOT:
        return new Climb(new ClimbIOSim());
      default:
        return new Climb(new ClimbIO() {});
    }
  }

  /**
   * Actual constructor for the subsystem. User should call the factory instead.
   *
   * @param io The IO layer to use for the motor.
   */
  private Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  /** Run the motor at a given speed */
  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Stop the motor, and lock the climber in place. */
  public void stop() {
    io.setSpeed(0.0);
    io.stop();
  }

  /**
   * Check if the climber is all the way retracted.
   *
   * @return True if the the climber is all the way retracted.
   */
  public boolean isAtBottom() {
    return io.isAtBottom();
  }
}
