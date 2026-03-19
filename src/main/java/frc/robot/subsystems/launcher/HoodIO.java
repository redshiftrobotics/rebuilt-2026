package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.launcher.LauncherConstants.HoodConstants;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Launcher subsystem's hood. */
public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public boolean isAdjustable = false;
    public double positionLeft = 0.0;
    public double positionRight = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void setAngle(Rotation2d launchAngle) {
    double radius = HoodConstants.HOOD_RADIUS.in(Meters);
    // Convert launch angle to hood angle
    Rotation2d hoodAngle = launchAngle.plus(Rotation2d.kCCW_90deg);
    Translation2d hoodPosition = new Translation2d(radius, hoodAngle);

    double lengthMeters = hoodPosition.getDistance(HoodConstants.LAUNCHER_AXLE_TO_ACTUATOR);

    double position =
        (lengthMeters - HoodConstants.ACTUATOR_LENGTH_MIN.in(Meters))
            / (HoodConstants.ACTUATOR_EXTENSION.in(Meters));
    setPosition(position);
  }

  public static Rotation2d getAngle(double position) {
    double actuatorLength =
        position * HoodConstants.ACTUATOR_EXTENSION.in(Meters)
            + HoodConstants.ACTUATOR_LENGTH_MIN.in(Meters);
    double radius = HoodConstants.HOOD_RADIUS.in(Meters);
    double actuatorDistance = HoodConstants.LAUNCHER_AXLE_TO_ACTUATOR.getNorm();

    // Law of cosines solved for angle
    // https://www.desmos.com/calculator/pkbsecs465
    Rotation2d relativeAngle =
        new Rotation2d(
            2 * Math.PI
                - Math.acos(
                    (radius * radius
                            + actuatorDistance * actuatorDistance
                            - (actuatorLength * actuatorDistance))
                        / (2 * radius * actuatorDistance)));
    // Convert from relative angle to absolute angle, then to launch angle
    return relativeAngle
        .plus(HoodConstants.LAUNCHER_AXLE_TO_ACTUATOR.getAngle())
        .plus(HoodConstants.ANGLE_ADJUSTMENT)
        .minus(Rotation2d.kCCW_90deg);
  }

  public default Rotation2d getAngle() {
    return getAngle(getPosition());
  }
}
