package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShotCalculator {
  public record ShotParameters(LinearVelocity velocity, Angle pitch, Rotation2d yaw) {}

  public static ShotParameters method1(
      Translation2d hubPosition, Translation2d robotVelocityMetersPerSecond) {
    // TODO implement using hub position for angle & adjusted hub position for velocity and yaw
    return calculateTrajectory(
        adjustedHubPosition(hubPosition, robotVelocityMetersPerSecond),
        robotVelocityMetersPerSecond);
  }

  private static Translation2d adjustedHubPosition(
      Translation2d hubPosition, Translation2d robotVelocityMetersPerSecond) {
    Translation2d adjustedHubPosition = hubPosition;
    for (int i = 0; i < 5; i++) {
      ShotParameters parameters =
          calculateTrajectory(adjustedHubPosition, robotVelocityMetersPerSecond);
      double time = timeOfFlight(parameters, adjustedHubPosition);
      // Shift hubPosition, not adjustedHubPosition, to avoid positive feedback
      adjustedHubPosition = shiftHubPosition(hubPosition, robotVelocityMetersPerSecond, time);
    }
    return adjustedHubPosition;
  }

  private static ShotParameters calculateTrajectory(
      Translation2d hubPosition, Translation2d robotVelocityMetersPerSecond) {
    double distance = hubPosition.getNorm() + LauncherConstants.LAUNCHER_X_OFFSET.in(Meters);
    // Formula acquired through experimentation
    double pitch = 75.0 - 0.5 * distance * Math.pow(Math.tanh(distance / 10.0), 4);
    // https://www.desmos.com/3d/enuvzskzsh
    double velocity =
        distance
            * Math.sqrt(
                9.81
                    / (2 * distance * Math.tan(pitch)
                        - LauncherConstants.HUB_Z_OFFSET.in(Meters)
                        + LauncherConstants.LAUNCHER_Z_OFFSET.in(Meters)))
            / Math.cos(pitch);

    return new ShotParameters(
        MetersPerSecond.of(velocity), Radians.of(pitch), hubPosition.getAngle());
  }

  private static double timeOfFlight(ShotParameters parameters, Translation2d hubPosition) {
    // Distance divided by horizontal shot speed
    return (hubPosition.getNorm() + LauncherConstants.LAUNCHER_X_OFFSET.in(Meters))
        / (Math.cos(parameters.pitch.in(Radians)) * parameters.velocity.in(MetersPerSecond));
  }

  private static Translation2d shiftHubPosition(
      Translation2d hubPosition, Translation2d robotVelocity, double timeOfFlight) {
    return hubPosition.plus(robotVelocity.times(timeOfFlight));
  }
}
