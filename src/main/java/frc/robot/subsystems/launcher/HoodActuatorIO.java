package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface HoodActuatorIO {
  @AutoLog
  public static class HoodActuatorIOInputs {
    public double position;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodActuatorIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void setAngle(Rotation2d angle) {
    Distance r = LauncherConstants.HOOD_RADIUS;
    Translation2d hoodPosition =
        new Translation2d(r.times(angle.getCos()), r.times(angle.getSin()));

    double lengthMeters = hoodPosition.getDistance(LauncherConstants.ACTUATOR_LOCATION);

    double position =
        (lengthMeters - LauncherConstants.ACTUATOR_LENGTH_MIN.in(Meters))
            / (LauncherConstants.ACTUATOR_EXTENSION.in(Meters));
    setPosition(position);
  }

  public default Rotation2d getAngle(Rotation2d angle) {
    double actuatorLength =
        getPosition() * LauncherConstants.ACTUATOR_EXTENSION.in(Meters)
            + LauncherConstants.ACTUATOR_LENGTH_MIN.in(Meters);
    double radius = LauncherConstants.HOOD_RADIUS.in(Meters);
    double actuatorDistance = LauncherConstants.ACTUATOR_LOCATION.getNorm();

    // https://www.desmos.com/calculator/pkbsecs465
    // https://www.youtube.com/watch?v=Qji5x8gBVX4
    Rotation2d relativeAngle =
        new Rotation2d(
            Math.acos(
                (radius * radius
                        + actuatorDistance * actuatorDistance
                        - (actuatorLength * actuatorDistance))
                    / (2 * radius * actuatorDistance)));

    return relativeAngle.plus(LauncherConstants.ACTUATOR_LOCATION.getAngle());
  }
}
