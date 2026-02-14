package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/**
 * The subsystem that controls the climber mechanism on the robot. Adapted from: {@link
 * https://github.com/redshiftrobotics/crescendo-2024/tree/main/src/main/java/frc/robot/subsystems/hang}
 *
 * @author Aceius E.
 */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  // Visualization:
  private final LoggedMechanism2d visualizerMechanism;
  private final LoggedMechanismLigament2d visualizerArm;

  /**
   * Creates a new Climb subsystem compatible with the given robot type.
   *
   * @param robotType The robot type to configure for.
   * @return An instance of the Climb subsystem with appropriate IO layers for the given robot.
   */
  public static Climb create(RobotType robotType) {
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

    visualizerMechanism = new LoggedMechanism2d(2, 3);
    visualizerArm =
        new LoggedMechanismLigament2d("ClimbArm", 2.0, 90.0, 10.0, new Color8Bit(Color.kBlue));

    visualizerMechanism.getRoot("ClimbRoot", 1, 0.5).append(visualizerArm);
  }

  @Override
  public void periodic() {
    // Log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    // Log visualization
    visualizerArm.setAngle(Units.radiansToDegrees(inputs.positionRad));
    Logger.recordOutput("Climb/Visualization", visualizerMechanism);
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
