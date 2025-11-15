package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.VirtualSubsystem;
import java.util.function.Supplier;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class DriveRotationController extends VirtualSubsystem {

  private final Drive drive;
  private Supplier<Rotation2d> goalSupplier;

  private int periodicCounter = 0;

  private final PIDController controller =
      new PIDController(
          HEADING_CONTROLLER_CONFIG.pid().kP(),
          HEADING_CONTROLLER_CONFIG.pid().kI(),
          HEADING_CONTROLLER_CONFIG.pid().kD());

  /**
   * Creates a new DriveRotationController.
   *
   * @param drive The drive subsystem to control.
   */
  public DriveRotationController(Drive drive) {
    this(drive, () -> null);
  }

  /**
   * Creates a new DriveRotationController. Holds last goal, which is kept when null is supplied,
   * and starts as the current heading.
   *
   * @param drive The drive subsystem to control.
   * @param goalSupplier A supplier that provides the desired goal heading. Can supply null if there
   *     is no new goal, in which case the controller will hold the last goal.
   */
  public DriveRotationController(Drive drive, Supplier<Rotation2d> goalSupplier) {
    this.drive = drive;
    this.goalSupplier = goalSupplier;

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(HEADING_CONTROLLER_CONFIG.toleranceRadians());

    reset();
  }

  @Override
  public void periodic() {
    periodicCounter += 1;
  }

  public void setGoalSupplier(Supplier<Rotation2d> goalSupplier) {
    this.goalSupplier = goalSupplier;
  }

  public void setGoal(Rotation2d goal) {
    controller.setSetpoint(goal.getRadians());
  }

  /**
   * Resets the heading controller to the current robot heading and omega speed.
   *
   * <p>This is typically called at the start of a new command to ensure the controller starts from
   * the current heading.
   */
  public void reset() {
    controller.setSetpoint(drive.getRobotPose().getRotation().getRadians());
    controller.reset();
  }

  /**
   * Calculates the output of the heading controller.
   *
   * @return The angular velocity command in radians per second.
   */
  public double calculate() {
    if (periodicCounter > 1) {
      reset();
    }

    Rotation2d goal = goalSupplier.get();

    if (goal != null) {
      controller.setSetpoint(goal.getRadians());
    }

    Rotation2d measured = drive.getRobotPose().getRotation();

    periodicCounter = 0;

    return controller.calculate(measured.getRadians());
  }

  /** Returns if the controller had a goal during the last calculate() call. */
  public boolean atGoal() {
    return controller.atSetpoint();
  }

  /** Returns the goal heading during the last calculate() call. */
  public Rotation2d getGoal() {
    return Rotation2d.fromRadians(controller.getSetpoint());
  }
}
