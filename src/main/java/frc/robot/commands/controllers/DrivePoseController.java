package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_TOLERANCE;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_TOLERANCE;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;

/** Controller for driving robot to goal pose using HolonomicDriveController */
public class DrivePoseController {

  private final Drive drive;

  private Pose2d goal;
  private Supplier<Pose2d> goalSupplier;

  private final HolonomicDriveController controller =
      new HolonomicDriveController(
          new PIDController(
              TRANSLATION_CONTROLLER_CONSTANTS.kP(),
              TRANSLATION_CONTROLLER_CONSTANTS.kI(),
              TRANSLATION_CONTROLLER_CONSTANTS.kD()),
          new PIDController(
              TRANSLATION_CONTROLLER_CONSTANTS.kP(),
              TRANSLATION_CONTROLLER_CONSTANTS.kI(),
              TRANSLATION_CONTROLLER_CONSTANTS.kD()),
          new ProfiledPIDController(
              ROTATION_CONTROLLER_CONSTANTS.kP(),
              ROTATION_CONTROLLER_CONSTANTS.kI(),
              ROTATION_CONTROLLER_CONSTANTS.kD(),
              DRIVE_CONFIG.getAngularConstraints()));

  /**
   * Creates a new DrivePoseController.
   *
   * @param drive The drive subsystem to control.
   */
  public DrivePoseController(Drive drive) {
    this(drive, () -> null);
  }

  /**
   * Creates a new DrivePoseController.
   *
   * @param drive The drive subsystem to control.
   * @param goalSupplier A supplier that provides the desired goal pose. Can supply null if
   *     there is no new goal, in which case the controller will hold the last goal.
   */
  public DrivePoseController(Drive drive, Supplier<Pose2d> goalSupplier) {
    this.drive = drive;
    this.goalSupplier = goalSupplier;

    controller.setTolerance(
        new Pose2d(TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, ROTATION_TOLERANCE));

    reset();
  }

  /**
   * Resets the pose controller to the current robot pose and speeds.
   *
   * <p>This is typically called at the start of a new command to ensure the controller starts from
   * the current pose.
   */
  public void reset() {
    goal = null;
    resetControllers();
  }

  public void setSetpointSupplier(Supplier<Pose2d> goalSupplier) {
    this.goalSupplier = goalSupplier;
  }

  public void setSetpoint(Pose2d goal) {
    this.goalSupplier = () -> goal;
  }

  private void resetControllers() {
    controller.getXController().reset();
    controller.getYController().reset();
    controller
        .getThetaController()
        .reset(
            drive.getRobotPose().getRotation().getRadians(),
            drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Calculates the required chassis speeds to drive to the goal pose.
   *
   * @return The desired chassis speeds.
   */
  public ChassisSpeeds calculate() {
    Optional<Pose2d> goal = getNextGoal();
    Pose2d measured = drive.getRobotPose();

    if (goal.isPresent()) {
      if (this.goal == null) {
        resetControllers();
      }
      this.goal = goal.get();
    }

    if (this.goal == null) {
      return new ChassisSpeeds();
    }

    return controller.calculate(measured, this.goal, 0, this.goal.getRotation());
  }

  /** Returns if the controller reached the goal during the last calculate() call. */
  public boolean atGoal() {
    return controller.atReference() && hasGoal();
  }

  /** Returns if the controller had a goal during the last calculate() call. */
  public boolean hasGoal() {
    return goal != null;
  }

  /** Returns the current goal from the supplier. */
  public Optional<Pose2d> getNextGoal() {
    return Optional.ofNullable(goalSupplier.get());
  }

  /** Returns the goal that was used in the last calculate() call. */
  public Optional<Pose2d> getLastGoal() {
    return Optional.ofNullable(goal);
  }
}
