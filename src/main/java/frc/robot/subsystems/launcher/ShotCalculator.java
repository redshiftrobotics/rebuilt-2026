package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShotCalculator {
  public record ShotParameters(LinearVelocity velocity, Rotation2d pitch, Rotation2d yaw) {}

  // Adjust distance, then calculate pitch, then calculate velocity based on pitch
  public static ShotParameters method1(
      Translation2d hubPosition, Translation2d robotVelocityMetersPerSecond, HoodType hoodType) {

    Translation2d adjustedHub = adjustedHubPosition(hubPosition, robotVelocityMetersPerSecond, hoodType);
    Distance adjustedDistance =
        Meters.of(adjustedHub.getNorm() - LauncherConstants.LAUNCHER_X_OFFSET.in(Meters));

    Rotation2d pitch = calculatePitch(adjustedDistance, hoodType);
    LinearVelocity velocity = calculateVelocity(adjustedDistance, pitch);
    return new ShotParameters(velocity, pitch, adjustedHub.getAngle());
  }

  private static Translation2d adjustedHubPosition(
      Translation2d hubPosition, Translation2d robotVelocityMetersPerSecond, HoodType hoodType) {
    Translation2d adjustedHubPosition = hubPosition;
    for (int i = 0; i < 5; i++) {
      ShotParameters parameters =
          calculateTrajectory(adjustedHubPosition, robotVelocityMetersPerSecond, hoodType);
      double time = timeOfFlight(parameters, adjustedHubPosition);
      // Shift hubPosition, not adjustedHubPosition, to avoid positive feedback
      adjustedHubPosition = shiftHubPosition(hubPosition, robotVelocityMetersPerSecond, time);
    }
    return adjustedHubPosition;
  }

  private static ShotParameters calculateTrajectory(
      Translation2d hubPosition, Translation2d robotVelocityMetersPerSecond, HoodType hoodType) {
    Distance distance =
        Meters.of(hubPosition.getNorm() - LauncherConstants.LAUNCHER_X_OFFSET.in(Meters));
    // Formula acquired through experimentation
    Rotation2d pitch = calculatePitch(distance, hoodType);

    return new ShotParameters(calculateVelocity(distance, pitch), pitch, hubPosition.getAngle());
  }

  static Rotation2d calculatePitch(Distance distance, HoodType hoodType) {
    return switch (hoodType) {
      case FIXED -> LauncherConstants.FIXED_LAUNCH_ANGLE;
        // Formula from experimentation
      case ACTUATOR -> Rotation2d.fromDegrees(
          75.0 - 15.0 * Math.tanh(2.0 * distance.in(Feet) / 25.0));
    };
  }

  static LinearVelocity calculateVelocity(Distance distance, Rotation2d pitch) {
    // https://www.desmos.com/3d/enuvzskzsh
    double velocity =
        distance.in(Meters)
            * Math.sqrt(
                9.81
                    / (2 * distance.in(Meters) * pitch.getTan()
                        - LauncherConstants.HUB_Z_OFFSET.in(Meters)
                        + LauncherConstants.LAUNCHER_Z_OFFSET.in(Meters)))
            / pitch.getCos();
    return MetersPerSecond.of(velocity);
  }

  private static double timeOfFlight(ShotParameters parameters, Translation2d hubPosition) {
    // Distance divided by horizontal shot speed
    return (hubPosition.getNorm() + LauncherConstants.LAUNCHER_X_OFFSET.in(Meters))
        / (parameters.pitch.getCos() * parameters.velocity.in(MetersPerSecond));
  }

  private static Translation2d shiftHubPosition(
      Translation2d hubPosition, Translation2d robotVelocity, double timeOfFlight) {
    return hubPosition.plus(robotVelocity.times(timeOfFlight));
  }
}
