package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumberGroup;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class DriveRotationController {
  private static final TunableNumberGroup factory = new TunableNumberGroup("HeadingController/");
  private static final TunablePID tunablePID = factory.pid("PID", HEADING_CONTROLLER_CONFIG.pid());
  private static final TunableNumber angularVelocity =
      factory.number("kAngularVelocity", DRIVE_CONFIG.maxAngularVelocity());
  private static final TunableNumber angularAcceleration =
      factory.number("kAngularAcceleration", DRIVE_CONFIG.maxAngularAcceleration());

  private final Drive drive;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          tunablePID.get().kP(),
          tunablePID.get().kI(),
          tunablePID.get().kD(),
          new TrapezoidProfile.Constraints(angularVelocity.get(), angularAcceleration.get()),
          Constants.LOOP_PERIOD_SECONDS);

  public DriveRotationController(Drive drive) {
    this.drive = drive;

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(HEADING_CONTROLLER_CONFIG.toleranceRadians());

    reset();
  }

  public void reset() {
    controller.reset(
        drive.getRobotPose().getRotation().getRadians(),
        drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  public void setGoal(Rotation2d goal) {
    controller.setGoal(goal.getRadians());
  }

  public double calculate() {
    return controller.calculate(drive.getRobotPose().getRotation().getRadians());
  }

  /** Returns if the controller reached the goal during the last calculate() call. */
  public boolean atGoal() {
    return controller.atGoal();
  }
}
