package frc.robot.commands.pipeline;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.DriveRotationController;
import frc.robot.utility.AllianceMirrorUtil;

/**
 * A mutable container for drive input values. This is a simple data class that stores linear
 * velocity, angular velocity or heading target, and field-relative settings.
 */
public class DriveInput {

  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double ANGLE_DEADBAND = 0.25;

  public static final double LINEAR_VELOCITY_EXPONENT = 2.0; // Square the joystick input
  public static final double ANGULAR_VELOCITY_EXPONENT = 2.0; // Square the joystick input

  public static final double SKEW_COMPENSATION_SCALAR = -0.03;

  private final Drive drive;

  private Translation2d linearVelocity = Translation2d.kZero;
  private double angularVelocity = 0.0;
  private boolean fieldRelative = true;
  private boolean headingTargeted = false;
  private Rotation2d headingTarget = null;

  public DriveInput(Drive drive) {
    this.drive = drive;
  }

  public ChassisSpeeds getChassisSpeeds(DriveRotationController headingController) {

    if (headingTarget != null) {
      headingController.setGoal(headingTarget);
    }

    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), angularVelocity);

    if (headingTargeted) {
      chassisSpeeds.omegaRadiansPerSecond = headingController.calculate();
    }

    // https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/drive/SwerveSubsystem.java#L381
    if (SKEW_COMPENSATION_SCALAR != 0 && Robot.isReal()) {
      Rotation2d skewCompensationFactor = Rotation2d
          .fromRadians(drive.getRobotSpeeds().omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR);
  
      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              chassisSpeeds, drive.getRobotPose().getRotation()),
          drive.getRobotPose().getRotation().plus(skewCompensationFactor));      
    }

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
    Vector<N2> linearVelocityVec = VecBuilder.fill(x, y);

    linearVelocityVec = MathUtil.applyDeadband(linearVelocityVec, JOYSTICK_DEADBAND);
    linearVelocityVec = MathUtil.copyDirectionPow(linearVelocityVec, LINEAR_VELOCITY_EXPONENT);
    linearVelocityVec = linearVelocityVec.times(drive.getMaxLinearSpeedMetersPerSec());

    return linearVelocity(new Translation2d(linearVelocityVec));
  }

  /**
   * Sets the angular velocity. This clears any heading target.
   *
   * @param angularVelocity The angular velocity in radians per second
   * @return This DriveInput for chaining
   */
  public DriveInput angularVelocity(double angularVelocity) {
    this.angularVelocity = angularVelocity;
    this.headingTargeted = false;
    return this;
  }

  /**
   * Sets the angular velocity based on a joystick input. This clears any heading target.
   *
   * @param omega The rotation joystick value (-1 to 1)
   * @return This DriveInput for chaining
   */
  public DriveInput angularVelocityStick(double omega) {

    omega = MathUtil.applyDeadband(omega, JOYSTICK_DEADBAND);
    omega = MathUtil.copyDirectionPow(omega, ANGULAR_VELOCITY_EXPONENT);
    omega = omega * drive.getMaxAngularSpeedRadPerSec();

    return angularVelocity(omega);
  }

  /**
   * Sets a heading target for the robot to face. This clears any angular velocity.
   *
   * @param headingTarget The desired heading angle, or null to clear the target
   * @return This DriveInput for chaining
   */
  public DriveInput headingTarget(Rotation2d headingTarget) {
    this.headingTarget = headingTarget;
    this.headingTargeted = true;
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

    if (translation.getNorm() < ANGLE_DEADBAND) {
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
