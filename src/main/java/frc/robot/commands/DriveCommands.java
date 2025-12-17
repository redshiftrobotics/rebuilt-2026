package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.DrivePoseController;
import frc.robot.subsystems.drive.controllers.DriveRotationController;
import java.util.function.Supplier;

public class DriveCommands {
  /** Drive to a pose, more precise */
  public static Command driveWithPoseController(Drive drive, Supplier<Pose2d> goalSupplier) {
    DrivePoseController controller = new DrivePoseController(drive);
    return drive
        .run(
            () -> {
              controller.setGoal(goalSupplier.get());
              drive.setRobotSpeeds(controller.calculate());
            })
        .until(controller::atGoal)
        .finallyDo(drive::stop);
  }

  public static Command rotateWithRotationController(
      Drive drive, Supplier<Rotation2d> goalSupplier) {
    DriveRotationController controller = new DriveRotationController(drive);
    return drive
        .run(
            () -> {
              controller.setGoal(goalSupplier.get());
              drive.setRobotSpeeds(new ChassisSpeeds(0.0, 0.0, controller.calculate()));
            })
        .until(controller::atGoal)
        .finallyDo(drive::stop);
  }

  /** Pathfind to a pose with pathplanner, only gets you roughly to target pose. */
  public static Command pathfindToPoseCommand(
      Drive drive, Pose2d desiredPose, double speedMultiplier, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(
        desiredPose, DRIVE_CONFIG.getPathConstraints(speedMultiplier), goalEndVelocity);
  }
}
