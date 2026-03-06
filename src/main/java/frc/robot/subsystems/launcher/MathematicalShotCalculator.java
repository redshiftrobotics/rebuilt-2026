package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.function.Function;

public class MathematicalShotCalculator {
  public record ShotParameters(LinearVelocity velocity, Rotation2d pitch) {
    @Override
    public final String toString() {
      return String.format(
          "ShotParameters(velocity=%.2f m/s, pitch=%.2f deg, yaw=%.2f deg)",
          velocity.in(MetersPerSecond), pitch.getDegrees());
    }
  }
  /**
   * @param hubPosition the position of the hub relative to the robot
   * @param robotVelocityMetersPerSecond the velocity of the robot
   * @param pitchProvider a function that takes the hub distance and returns the desired shot pitch
   * @return the position of the hub accounting for robot velocity
   */
  public static Translation2d adjustedHubPosition(
      Translation2d hubPosition,
      Translation2d robotVelocityMetersPerSecond,
      Function<Double, Rotation2d> pitchProvider) {
    Translation2d adjustedHubPosition = hubPosition;
    for (int i = 0; i < 5; i++) {
      Rotation2d pitch = pitchProvider.apply(adjustedHubPosition.getNorm());
      ShotParameters parameters =
          new ShotParameters(calculateVelocity(adjustedHubPosition.getNorm(), pitch), pitch);
      double time = timeOfFlight(parameters, adjustedHubPosition);
      // Shift hubPosition, not adjustedHubPosition, to avoid positive feedback
      adjustedHubPosition = hubPosition.minus(robotVelocityMetersPerSecond.times(time));
    }
    return adjustedHubPosition;
  }

  public static LinearVelocity calculateVelocity(double distanceMeters, Rotation2d pitch) {
    // https://www.desmos.com/3d/enuvzskzsh
    double velocity =
        distanceMeters
            * Math.sqrt(
                9.81
                    / (2
                        * (distanceMeters * pitch.getTan()
                            - LauncherConstants.HUB_Z_OFFSET.in(Meters)
                            + LauncherConstants.LAUNCHER_Z_OFFSET.in(Meters))))
            / pitch.getCos();
    return MetersPerSecond.of(velocity);
  }

  public static Rotation2d calculatePitch(double distanceMeters) {
    // Formula from experimentation
    return Rotation2d.fromDegrees(
        75.0 - 15.0 * Math.tanh(2.0 * Units.metersToFeet(distanceMeters) / 25.0));
  }

  public static double timeOfFlight(ShotParameters parameters, Translation2d hubPosition) {
    // Distance divided by horizontal shot speed
    return (hubPosition.getNorm() + LauncherConstants.LAUNCHER_X_OFFSET.in(Meters))
        / (parameters.pitch.getCos() * parameters.velocity.in(MetersPerSecond));
  }
}
