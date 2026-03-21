package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.geometry.AllianceMirrorUtil;

public class SelfDrivingCommands {
    // https://pathplanner.dev/pplib-pathfinding.html#pathfind-to-pose

    static final PathConstraints constraints =
            new PathConstraints(5.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    static final double endVelocity1 = 0.2;
    static final double endVelocity2 = constraints.maxVelocityMPS() * 0.9;

    static final double trenchStartX = 3.500;
    static final double trenchEndX = 5.750;
    static final double trenchLeftY =
            (FieldConstants.LinesHorizontal.leftTrenchOpenEnd + FieldConstants.LinesHorizontal.leftTrenchOpenStart) / 2;
    static final double trenchRightY =
            (FieldConstants.LinesHorizontal.rightTrenchOpenStart + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
                    / 2;

    public static Command selfDriveToOtherZone(Drive drive) {
        return Commands.either(
                        selfDriveToAllianceZone(drive),
                        selfDriveToNeutralZone(drive),
                        () -> isInNeutralZone(drive.getRobotPose()))
                .withName("Drive to other zone");
    }

    public static Command selfDriveToAllianceZone(Drive drive) {
        return Commands.either(
                        thoughTrench(trenchLeftY, true),
                        thoughTrench(trenchRightY, true),
                        () -> isLeftSide(drive.getRobotPose()))
                .andThen(AutoBuilder.pathfindToPoseFlipped(
                        new Pose2d(FieldConstants.fieldLength / 6, FieldConstants.fieldWidth / 2, Rotation2d.kZero),
                        constraints));
    }

    public static Command selfDriveToNeutralZone(Drive drive) {
        return Commands.either(
                        thoughTrench(trenchLeftY, false),
                        thoughTrench(trenchRightY, false),
                        () -> isLeftSide(drive.getRobotPose()))
                .andThen(AutoBuilder.pathfindToPoseFlipped(
                        new Pose2d(FieldConstants.fieldLength / 2.5, FieldConstants.fieldWidth / 2, Rotation2d.kZero),
                        constraints));
    }

    private static Command thoughTrench(double y, boolean reverse) {
        return AutoBuilder.pathfindToPoseFlipped(
                        new Pose2d(reverse ? trenchEndX : trenchStartX, y, Rotation2d.kZero), constraints, endVelocity1)
                .andThen(AutoBuilder.pathfindToPoseFlipped(
                        new Pose2d(reverse ? trenchStartX : trenchEndX, y, Rotation2d.kZero),
                        constraints,
                        endVelocity2));
    }

    private static boolean isInNeutralZone(Pose2d pose) {
        return AllianceMirrorUtil.applyX(pose.getTranslation().getX()) > FieldConstants.LinesVertical.hubCenter;
    }

    private static boolean isLeftSide(Pose2d pose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return pose.getY() >= FieldConstants.LinesHorizontal.center;
        } else {
            return pose.getY() < FieldConstants.LinesHorizontal.center;
        }
    }

    private SelfDrivingCommands() {
        System.err.println("SelfDrivingCommands is a utility class, not to be instantiated!");
    }
}
