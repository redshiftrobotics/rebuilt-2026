package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.geometry.AllianceMirrorUtil;
import java.util.Set;

public class SelfDrivingCommands {
  static PathConstraints constraints =
      new PathConstraints(
          DRIVE_CONFIG.maxLinearVelocity(),
          4.0,
          Units.degreesToRadians(540),
          Units.degreesToRadians(720));

  public static Command selfDriveToOtherZone(Drive drive) {
    return Commands.either(
            Commands.defer(SelfDrivingCommands::selfDriveToAllianceZone, Set.of(drive)),
            Commands.defer(SelfDrivingCommands::selfDriveToNeutralZone, Set.of(drive)),
            () -> isInNeutralZone(drive.getRobotPose()))
        .withName("Drive to other zone");
  }

  public static Command selfDriveToAllianceZone() {
    return AutoBuilder.pathfindToPose(
        AllianceMirrorUtil.apply(
            new Pose2d(
                FieldConstants.fieldLength / 6, FieldConstants.fieldWidth / 2, Rotation2d.kZero)),
        constraints);
  }

  public static Command selfDriveToNeutralZone() {
    return AutoBuilder.pathfindToPose(
        AllianceMirrorUtil.apply(
            new Pose2d(
                FieldConstants.fieldLength / 2.5, FieldConstants.fieldWidth / 2, Rotation2d.kZero)),
        constraints);
  }

  private static boolean isInNeutralZone(Pose2d pose) {
    return MathUtil.isNear(
        FieldConstants.LinesVertical.center, pose.getX(), Units.inchesToMeters(120));
  }

  private SelfDrivingCommands() {
    System.err.println("SelfDrivingCommands is a utility class, not to be instantiated!");
  }
}
