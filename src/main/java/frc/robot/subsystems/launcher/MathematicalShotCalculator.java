package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.FieldConstants;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import frc.robot.subsystems.launcher.LauncherConstants.LauncherMathConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class MathematicalShotCalculator {
  public record MathematicalShotParameters(LinearVelocity velocity, Rotation2d pitch, Rotation2d yaw) {
    public MathematicalShotParameters(LinearVelocity velocity, Rotation2d pitch) {
      this(velocity, pitch, Rotation2d.kZero);
    }
    @Override
    public final String toString() {
      return String.format(
          "ShotParameters(velocity=%.2f m/s, pitch=%.2f deg, yaw=%.2f deg)",
          velocity.in(MetersPerSecond), pitch.getDegrees(), yaw.getDegrees());
    }

    public double getWheelRadPerSec() {
      return velocity.in(MetersPerSecond) / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters);
    }

    public double getHoodPosition() {
      return getHoodPositionFromAngle(pitch);
    }
  }

  public static MathematicalShotParameters calculateAndSetShot(
      Pose2d robotPose, ChassisSpeeds robotRelativeVelocity, boolean isHoodAdjustable) {
    
      Function<Double, Rotation2d> pitchCalculator = isHoodAdjustable ? MathematicalShotCalculator::calculatePitch : d -> LauncherMathConstants.FIXED_LAUNCH_ANGLE;

      Translation2d hubLocation =
          FieldConstants.Hub.topCenterPoint.toTranslation2d().minus(robotPose.getTranslation());

      ChassisSpeeds fieldRelativeSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotPose.getRotation());

      Translation2d hubTranslation =
          MathematicalShotCalculator.adjustedHubPosition(
              hubLocation,
              new Translation2d(
                  fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond),
              pitchCalculator);
      Logger.recordOutput(
          "MathematicalLauncherCalculator/HubAdjustment", hubTranslation.minus(hubLocation).getNorm());

      double distance = hubTranslation.getNorm();
      Rotation2d runningHoodPitch = pitchCalculator.apply(distance);

      MathematicalShotParameters parameters =
          new MathematicalShotParameters(
              MathematicalShotCalculator.calculateVelocity(hubLocation.getNorm(), runningHoodPitch),
              runningHoodPitch, hubTranslation.getAngle());

      double timeOfFlight = MathematicalShotCalculator.timeOfFlight(parameters, hubLocation);
      Logger.recordOutput("MathematicalLauncherCalculator/timeOfFlight", Math.round(timeOfFlight * 10.0) / 10.0);

      LinearVelocity ballLinearVelocity =
          MathematicalShotCalculator.calculateVelocity(distance, runningHoodPitch);
      ballLinearVelocity = ballLinearVelocity.times(LauncherMathConstants.LAUNCHER_VELOCITY_MULTIPLIER.get());

      return new MathematicalShotParameters(ballLinearVelocity, runningHoodPitch, hubTranslation.getAngle());
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
      MathematicalShotParameters parameters =
          new MathematicalShotParameters(calculateVelocity(adjustedHubPosition.getNorm(), pitch), pitch);
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
                            - LauncherMathConstants.HUB_Z_OFFSET.in(Meters)
                            + LauncherMathConstants.LAUNCHER_Z_OFFSET.in(Meters))))
            / pitch.getCos();
    return MetersPerSecond.of(velocity);
  }

  public static Rotation2d calculatePitch(double distanceMeters) {
    // Formula from experimentation
    return Rotation2d.fromDegrees(
        75.0 - 15.0 * Math.tanh(2.0 * Units.metersToFeet(distanceMeters) / 25.0));
  }

  public static double timeOfFlight(MathematicalShotParameters parameters, Translation2d hubPosition) {
    // Distance divided by horizontal shot speed
    return (hubPosition.getNorm() + LauncherMathConstants.LAUNCHER_X_OFFSET.in(Meters))
        / (parameters.pitch.getCos() * parameters.velocity.in(MetersPerSecond));
  }

  public static double getHoodPositionFromAngle(Rotation2d angle) {
    Distance r = LauncherMathConstants.HOOD_RADIUS;
    // Convert launch angle to hood angle
    angle = angle.plus(Rotation2d.kCCW_90deg);
    Translation2d hoodPosition =
        new Translation2d(r.times(angle.getCos()), r.times(angle.getSin()));

    double lengthMeters = hoodPosition.getDistance(LauncherMathConstants.ACTUATOR_LOCATION);

    double position =
        (lengthMeters - LauncherMathConstants.ACTUATOR_LENGTH_MIN.in(Meters))
            / (LauncherMathConstants.ACTUATOR_EXTENSION.in(Meters));
    return position;
  }

  public static Rotation2d getAngleFromHoodPosition(double position) {
    double actuatorLength =
        position * LauncherMathConstants.ACTUATOR_EXTENSION.in(Meters)
            + LauncherMathConstants.ACTUATOR_LENGTH_MIN.in(Meters);
    double radius = LauncherMathConstants.HOOD_RADIUS.in(Meters);
    double actuatorDistance = LauncherMathConstants.ACTUATOR_LOCATION.getNorm();

    // https://www.desmos.com/calculator/pkbsecs465
    // https://www.youtube.com/watch?v=Qji5x8gBVX4
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
        .plus(LauncherMathConstants.ACTUATOR_LOCATION.getAngle())
        .minus(Rotation2d.kCCW_90deg);
  }
}