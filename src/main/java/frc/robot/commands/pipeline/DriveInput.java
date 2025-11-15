package frc.robot.commands.pipeline;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceMirrorUtil;

/**
 * A mutable container for drive input values. This is a simple data class that stores linear
 * velocity, angular velocity or heading target, and field-relative settings.
 */
public class DriveInput {

  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double ANGLE_DEADBAND = 0.5;

  public static final double LINEAR_VELOCITY_EXPONENT = 2.0; // Square the joystick input
  public static final double ANGULAR_VELOCITY_EXPONENT = 2.0; // Square the joystick input

  private final Drive drive;

  private Translation2d linearVelocity = Translation2d.kZero;
  private double angularVelocity = 0.0;
  private boolean fieldRelative = true;

  public DriveInput(Drive drive) {
    this.drive = drive;
  }

  public ChassisSpeeds getChassisSpeeds() {

    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), angularVelocity);

    if (fieldRelative) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              chassisSpeeds, AllianceMirrorUtil.apply(drive.getRobotPose().getRotation()));
    }

    return chassisSpeeds;
  }

  /**
   * Sets the linear velocity.
   *
   * @param linearVelocity The linear velocity in meters per second
   * @return This DriveInput for chaining
   */
  public DriveInput linearVelocity(Translation2d linearVelocity) {
    this.linearVelocity = linearVelocity;
    return this;
  }

  /**
   * Sets the linear velocity based on joystick inputs.
   *
   * @param x The x-axis joystick value (-1 to 1)
   * @param y The y-axis joystick value (-1 to 1)
   * @return This DriveInput for chaining
   */
  public DriveInput linearVelocityStick(double x, double y) {
    Translation2d translation = new Translation2d(x, y);

    double magnitude = MathUtil.applyDeadband(translation.getNorm(), JOYSTICK_DEADBAND);

    if (magnitude == 0) return linearVelocity(Translation2d.kZero);

    double magnitudeSquared = Math.abs(Math.pow(magnitude, LINEAR_VELOCITY_EXPONENT));

    Translation2d squaredTranslation = new Translation2d(magnitudeSquared, translation.getAngle());

    this.linearVelocity = squaredTranslation.times(drive.getMaxLinearSpeedMetersPerSec());

    return this;
  }

  /**
   * Sets the angular velocity. This clears any heading target.
   *
   * @param angularVelocity The angular velocity in radians per second
   * @return This DriveInput for chaining
   */
  public DriveInput angularVelocity(double angularVelocity) {
    this.angularVelocity = angularVelocity;
    return this;
  }

  /**
   * Sets the angular velocity based on a joystick input. This clears any heading target.
   *
   * @param omega The rotation joystick value (-1 to 1)
   * @return This DriveInput for chaining
   */
  public DriveInput angularVelocityStick(double omega) {
    double deadbandOmega = MathUtil.applyDeadband(omega, JOYSTICK_DEADBAND);

    double omegaSquared = Math.copySign(Math.pow(deadbandOmega, ANGULAR_VELOCITY_EXPONENT), omega);

    return angularVelocity(omegaSquared * drive.getMaxAngularSpeedRadPerSec());
  }

  /**
   * Sets a heading target for the robot to face. This clears any angular velocity.
   *
   * @param headingTarget The desired heading angle, or null to clear the target
   * @return This DriveInput for chaining
   */
  public DriveInput headingTarget(Rotation2d headingTarget) {
    if (headingTarget != null) {
      drive.getHeadingController().setGoal(headingTarget);
    }
    this.angularVelocity = drive.getHeadingController().calculate();
    return this;
  }

  /**
   * Sets a heading target for the robot to face. This clears any angular velocity.
   *
   * @param headingTarget The desired heading angle, or null to clear the target
   * @param allianceRelative Whether the heading target is relative to the alliance color
   * @return This DriveInput for chaining
   */
  public DriveInput headingTarget(Rotation2d headingTarget, boolean allianceRelative) {
    if (allianceRelative && headingTarget != null) {
      headingTarget = AllianceMirrorUtil.apply(headingTarget);
    }
    return headingTarget(headingTarget);
  }

  /**
   * Sets a heading target based on a joystick input. If the joystick is in the deadzone, this does
   * nothing.
   *
   * @param x The x-axis joystick value (-1 to 1)
   * @param y The y-axis joystick value (-1 to 1)
   * @return This DriveInput for chaining
   */
  public DriveInput headingStick(double x, double y) {
    Translation2d translation = new Translation2d(x, y);

    if (translation.getNorm() < JOYSTICK_DEADBAND) {
      return headingTarget(null);
    }

    return headingTarget(translation.getAngle(), true);
  }

  /**
   * Sets a heading target to face a specific point on the field.
   *
   * @param point The point to face
   * @return This DriveInput for chaining
   */
  public DriveInput facingPoint(Translation2d point) {
    if (point == null) {
      return this;
    }

    Translation2d diff = point.minus(drive.getRobotPose().getTranslation());

    if (diff.getNorm() < Units.inchesToMeters(1)) {
      return this;
    }

    return headingTarget(diff.getAngle());
  }

  /**
   * Enables field-relative mode (translation commands are relative to the field).
   *
   * @return This DriveInput for chaining
   */
  public DriveInput fieldRelativeEnabled() {
    this.fieldRelative = true;
    return this;
  }

  /**
   * Disables field-relative mode (translation commands are relative to the robot).
   *
   * @return This DriveInput for chaining
   */
  public DriveInput fieldRelativeDisabled() {
    this.fieldRelative = false;
    return this;
  }

  /**
   * Scales the linear and angular velocities by the given coefficients.
   *
   * @param linearCoeff The coefficient to scale the linear velocity
   * @param angularCoeff The coefficient to scale the angular velocity
   * @return This DriveInput for chaining
   */
  public DriveInput coefficients(double linearCoeff, double angularCoeff) {
    this.linearVelocity = this.linearVelocity.times(linearCoeff);
    this.angularVelocity *= angularCoeff;
    return this;
  }
}
