package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumberGroup;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class DriveRotationController {
  private static final TunableNumberGroup factory = new TunableNumberGroup("HeadingController/");

  private static final TunablePID PID = factory.pid("PID", HEADING_CONTROLLER_CONFIG.pid());
  private static final TunableNumber tolerence =
      factory.number("tolerenceDegrees", HEADING_CONTROLLER_CONFIG.tolerance().getDegrees());

  private static final TunableNumber angularVelocity =
      factory.number("kAngularVelocity", DRIVE_CONFIG.maxAngularVelocity());
  private static final TunableNumber angularAcceleration =
      factory.number("kAngularAcceleration", DRIVE_CONFIG.maxAngularAcceleration());

  private final Drive drive;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          PID.get().kP(),
          PID.get().kI(),
          PID.get().kD(),
          new TrapezoidProfile.Constraints(angularVelocity.get(), angularAcceleration.get()),
          Constants.LOOP_PERIOD_SECONDS);

  public DriveRotationController(Drive drive) {
    this.drive = drive;

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(
        Units.degreesToRadians(tolerence.get()),
        HEADING_CONTROLLER_CONFIG.velocityTolerence().getRadians());

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

    PID.ifChanged(hashCode(), pid -> controller.setPID(pid.kP(), pid.kI(), pid.kD()));
    tolerence.ifChanged(hashCode(), controller::setTolerance);
    TunableNumber.ifChanged(
        hashCode(),
        values -> controller.setConstraints(new Constraints(values[0], values[1])),
        angularVelocity,
        angularAcceleration);

    return controller.calculate(drive.getRobotPose().getRotation().getRadians());
  }

  /** Returns if the controller reached the goal during the last calculate() call. */
  public boolean atGoal() {
    return controller.atGoal();
  }
}
