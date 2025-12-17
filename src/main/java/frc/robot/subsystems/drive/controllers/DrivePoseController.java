package frc.robot.subsystems.drive.controllers;

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

/** Controller for driving robot to goal pose using HolonomicDriveController */
public class DrivePoseController {

  private final Drive drive;

  private Pose2d goal = null;

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

  public DrivePoseController(Drive drive) {
    this.drive = drive;

    controller.setTolerance(
        new Pose2d(TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, ROTATION_TOLERANCE));

    reset();
  }

  public void reset() {
    controller.getXController().reset();
    controller.getYController().reset();
    controller
        .getThetaController()
        .reset(
            drive.getRobotPose().getRotation().getRadians(),
            drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  public void setGoal(Pose2d goal) {
    this.goal = goal;
  }

  /**
   * Calculates the required chassis speeds to drive to the goal pose.
   *
   * @return The desired chassis speeds.
   */
  public ChassisSpeeds calculate() {
    return controller.calculate(drive.getRobotPose(), this.goal, 0, this.goal.getRotation());
  }

  /** Returns if the controller reached the goal during the last calculate() call. */
  public boolean atGoal() {
    return controller.atReference() && hasGoal();
  }

  /** Returns if the controller had a goal during the last calculate() call. */
  public boolean hasGoal() {
    return goal != null;
  }
}
