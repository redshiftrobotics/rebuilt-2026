package frc.robot.commands;

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

public class SelfDrivingCommands {
  // TODO Copied from docs
  static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public static Command selfDriveToOtherZone(Drive drivetrain) {
    return Commands.either(
            selfDriveToAllianceZone(),
            selfDriveToNeutralZone(),
            () -> isInNeutralZone(drivetrain.getRobotPose()))
        .withName("Drive to other zone");
  }

  public static Command selfDriveToAllianceZone() {
    return AutoBuilder.pathfindToPose(
            new Pose2d( // TODO WRONG POSE
                FieldConstants.fieldWidth / 2, FieldConstants.fieldLength / 2, Rotation2d.kZero),
            constraints)
        .alongWith(Commands.run(() -> System.out.println("DriveToAllianceZone")));
  }

  public static Command selfDriveToNeutralZone() {
    return AutoBuilder.pathfindToPose(
            new Pose2d(
                FieldConstants.fieldWidth / 2, FieldConstants.fieldLength / 2, Rotation2d.kZero),
            constraints)
        .alongWith(Commands.run(() -> System.out.println("DriveToNeutralZone")));
  }

  private static boolean isInNeutralZone(Pose2d pose) {
    return MathUtil.isNear(
        FieldConstants.LinesVertical.center, pose.getX(), Units.inchesToMeters(120));
  }

  private SelfDrivingCommands() {
    System.err.println("SelfDrivingCommands is a utility class, not to be instantiated!");
  }
}
