package frc.robot.commands.pipeline;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.controllers.DriveRotationController;
import frc.robot.utility.AllianceMirrorUtil;
import java.util.Optional;

/**
 * A mutable container for drive input values. This is a simple data class that stores linear
 * velocity, angular velocity or heading target, and field-relative settings.
 */
public class DriveInput {

  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double JOYSTICK_ANGLE_DEADBAND = 0.8;

  public static final double LINEAR_VELOCITY_EXPONENT = 1.75;
  public static final double ANGULAR_VELOCITY_EXPONENT = 2;

  private Translation2d linearVelocity = Translation2d.kZero;
  private double angularVelocity = 0.0;
  private Optional<Rotation2d> headingTarget = Optional.empty();
  private boolean holdHeading = false;

  private boolean fieldRelative = false;
  private boolean locustMode = false;

  public ChassisSpeeds getChassisSpeeds(
      Rotation2d robotHeading, DriveRotationController headingController) {

    applyLocustMode(robotHeading);

    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), angularVelocity);

    headingTarget.ifPresent(headingController::setGoal);
    if (headingTarget.isPresent() || holdHeading) {
      double raw = headingController.calculate(); // must call calculate to update atGoal()
      chassisSpeeds.omegaRadiansPerSecond = headingController.atGoal() ? 0 : raw;
    }

    if (fieldRelative) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              chassisSpeeds, AllianceMirrorUtil.apply(robotHeading));
    }

    return chassisSpeeds;
  }

  /**
   * Sets the linear velocity.
   *
   * @param linearVelocity The linear velocity in meters per second
   * @return this
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
   * @return this
   */
  public DriveInput linearVelocityStick(double x, double y, double maxSpeed) {
    Vector<N2> linearVelocityVec = VecBuilder.fill(x, y);

    linearVelocityVec = MathUtil.applyDeadband(linearVelocityVec, JOYSTICK_DEADBAND);
    linearVelocityVec = MathUtil.copyDirectionPow(linearVelocityVec, LINEAR_VELOCITY_EXPONENT);
    linearVelocityVec = linearVelocityVec.times(maxSpeed);

    return linearVelocity(new Translation2d(linearVelocityVec));
  }

  /**
   * Sets the angular velocity. This clears any heading target.
   *
   * @param angularVelocity The angular velocity in radians per second
   * @return this
   */
  public DriveInput angularVelocity(double angularVelocity) {
    this.angularVelocity = angularVelocity;
    this.headingTarget = Optional.empty();
    this.holdHeading = false;
    return this;
  }

  /**
   * Sets the angular velocity based on a joystick input. This clears any heading target.
   *
   * @param omega The rotation joystick value (-1 to 1)
   * @return this
   */
  public DriveInput angularVelocityStick(double omega, double maxSpeed) {

    omega = MathUtil.applyDeadband(omega, JOYSTICK_DEADBAND);
    omega = MathUtil.copyDirectionPow(omega, ANGULAR_VELOCITY_EXPONENT);
    omega = omega * maxSpeed;

    return angularVelocity(omega);
  }

  /**
   * Sets a heading target for the robot to face. This clears any angular velocity.
   *
   * @param headingTarget The desired heading angle, or null to clear the target
   * @return this
   */
  public DriveInput headingTarget(Rotation2d headingTarget) {
    this.headingTarget = Optional.of(headingTarget);
    this.angularVelocity = 0.0;
    return this;
  }

  /**
   * Clears the heading target and stops any angular velocity.
   *
   * @return this
   */
  public DriveInput holdHeading() {
    this.holdHeading = true;
    return this;
  }

  /**
   * Sets a heading target for the robot to face. This clears any angular velocity.
   *
   * @param headingTarget The desired heading angle, or null to clear the target
   * @param allianceRelative Whether the heading target is relative to the alliance color
   * @return this
   */
  public DriveInput headingTarget(Rotation2d headingTarget, boolean allianceRelative) {
    if (allianceRelative) {
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
   * @return this
   */
  public DriveInput headingStick(double x, double y) {
    Translation2d translation = new Translation2d(x, y);

    if (translation.getNorm() < JOYSTICK_ANGLE_DEADBAND) {
      return holdHeading();
    }

    return headingTarget(translation.getAngle(), true);
  }

  /**
   * Sets a heading target to face a specific point on the field.
   *
   * @param point The point to face
   * @return this
   */
  public DriveInput facingPoint(Translation2d point, Pose2d robotPose) {
    Translation2d diff = point.minus(robotPose.getTranslation());

    if (diff.getNorm() < Units.inchesToMeters(1)) {
      return angularVelocity(0);
    }

    return headingTarget(diff.getAngle());
  }

  /**
   * Tries to aim the drive in the direction of linear motion. Scales linear velocity to help with
   * this.
   *
   * @return this
   */
  public DriveInput locustMode() {
    this.locustMode = true;
    return this;
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
   * Scales the linear velocities by the given coefficients.
   *
   * @param The coefficient to scale the linear velocity
   * @return this
   */
  public DriveInput linearCoefficient(double scaler) {
    linearVelocity = linearVelocity.times(scaler);
    return this;
  }

  /**
   * Scales the linear and angular velocities by the given coefficients.
   *
   * @param scaler The coefficient to scale the angular velocity
   * @return this
   */
  public DriveInput angularCoefficient(double scaler) {
    angularVelocity = angularVelocity * scaler;
    return this;
  }

  // --- Mode Specific Methods ---

  private DriveInput applyLocustMode(Rotation2d robotHeading) {

    if (!locustMode) {
      return this;
    }

    if (!fieldRelative) {
      Translation2d translation = linearVelocity;
      return linearVelocity(new Translation2d(translation.getNorm(), 0))
          .angularVelocity(translation.getY());
    }

    if (linearVelocity.getNorm() < 0.25) {
      return angularVelocity(0);
    }

    Rotation2d targetAngle = linearVelocity.getAngle();

    // If already facing the right way: cosine = 1, scale = 1
    // If sideways: cosine = 0, scale = 0
    // If backwards: cosine = -1, scale = -0.05
    double cosign = targetAngle.minus(robotHeading).getCos();

    double scale = Math.max(cosign, cosign * 0.05);

    linearVelocity = linearVelocity.times(scale);

    return headingTarget(targetAngle);
  }
}
