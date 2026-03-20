package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperRunMode;
import frc.robot.subsystems.launcher.LaunchCalculator;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.utility.geometry.AllianceMirrorUtil;
import frc.robot.utility.geometry.GeomUtil;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumberGroup;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LaunchCommands {
    private static final ChassisSpeeds ZERO_CHASSIS_SPEEDS = new ChassisSpeeds();

    private static final TunableNumberGroup launchingGroup = new TunableNumberGroup("LaunchCalculator/Driving");

    private static final TunableNumber driveLaunchKp = launchingGroup.number("kP", 8.0);
    private static final TunableNumber driveLaunchKd = launchingGroup.number("kD", 0.5);
    private static final TunableNumber driveControllerYawToleranceDeg =
            launchingGroup.number("ControllerYawToleranceDeg", 0.01);

    private static final TunableNumber driveYawLaunchToleranceDeg = launchingGroup.number("YawToleranceDeg", 5.0);
    private static final TunableNumber driveLaunchMaxPolarVelocityRadPerSec =
            launchingGroup.number("MaxPolarVelocityRadPerSec", 0.6);
    private static final TunableNumber driveLauncherCORMinErrorDeg =
            launchingGroup.number("DriveLauncherCORMinErrorDeg", 15.0);
    private static final TunableNumber driveLauncherCORMaxErrorDeg =
            launchingGroup.number("DriveLauncherCORMaxErrorDeg", 30.0);

    public static Command primeToLaunch(Drive drive, Launcher launcher) {
        return Commands.parallel(
                DriveCommands.rotateWithRotationController(drive, () -> launcher.getRobotYaw()),
                launcher.runOnce(() -> {
                    launcher.start();
                }));
    }

    public static Command launchInPlace(Drive drive, Launcher launcher, Hopper hopper) {
        Debouncer alignedWithHubDebouncer = new Debouncer(1);
        BooleanSupplier isAligned = () -> alignedWithHubDebouncer.calculate((MathUtil.isNear(
                LaunchCalculator.getInstance()
                        .getParameters(drive.getRobotPose(), ZERO_CHASSIS_SPEEDS)
                        .driveAngle()
                        .getDegrees(),
                drive.getRobotPose().getRotation().getDegrees(),
                2.0)));

        Command autoAlign = driveWhileLaunching(drive, () -> ZERO_CHASSIS_SPEEDS)
                .until(isAligned)
                .withTimeout(8);

        Command launchFuel = Commands.parallel(
                launcher.runOnce(launcher::start),
                Commands.sequence(
                        hopper.runOnce(() -> hopper.setMode(HopperRunMode.PREP_SHOT)),
                        Commands.waitUntil(launcher::isReadyDebounced),
                        hopper.runOnce(() -> hopper.setMode(HopperRunMode.FIRING))));

        return Commands.parallel(launchFuel, autoAlign)
                .finallyDo((interrupted) -> {
                    hopper.setMode(HopperRunMode.STOPPED);
                    launcher.stop();
                })
                .withName("Launching in place");
    }

    public static Command driveWhileLaunching(
            Drive drive, final Supplier<ChassisSpeeds> initialRobotRelativeSpeedsSupplier) {
        return drive.run(() -> {
            final Pose2d robotPose = drive.getRobotPose();
            final ChassisSpeeds measuredRobotRelativeSpeeds = drive.getRobotSpeeds();
            final Rotation2d measuredRobotAngle = robotPose.getRotation();

            // Run PID controller
            final LaunchingParameters parameters =
                    LaunchCalculator.getInstance().getParameters(robotPose, measuredRobotRelativeSpeeds);

            double omegaOutput = parameters.driveAngularVelocityRadPerSec()
                    + (parameters.driveAngle().minus(measuredRobotAngle).getRadians() * driveLaunchKp.get())
                    + ((parameters.driveAngularVelocityRadPerSec() - measuredRobotRelativeSpeeds.omegaRadiansPerSecond)
                            * driveLaunchKd.get());

            // within tolerance, no need to drive
            if (MathUtil.isNear(
                    parameters.driveAngle().getDegrees(),
                    measuredRobotAngle.getDegrees(),
                    driveControllerYawToleranceDeg.get())) {
                omegaOutput = 0.0;
            }

            ChassisSpeeds initialFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    initialRobotRelativeSpeedsSupplier.get(), AllianceMirrorUtil.apply(measuredRobotAngle));
            Translation2d initialLinearVelocity = new Translation2d(
                    initialFieldRelativeSpeeds.vxMetersPerSecond, initialFieldRelativeSpeeds.vyMetersPerSecond);

            // Only limit if launching, not passing
            if (!parameters.passing()) {
                // Calculate max linear velocity magnitude based on the max polar velocity
                // Basically, if the robot is moving (linearly) faster than it can rotate
                // to correct its angle to the hub, we cap the velocity so that it can always
                // face the
                // hub
                double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
                double robotDriveAngle =
                        Math.abs(AllianceMirrorUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                                .minus(robotPose.getTranslation())
                                .getAngle()
                                .minus(initialLinearVelocity.getAngle())
                                .getRadians());
                double robotHubDistance = parameters.distanceNoLookahead();
                double hubAngle =
                        driveLaunchMaxPolarVelocityRadPerSec.get() * LaunchCalculator.getTimeOfFlight(robotHubDistance);
                double lookaheadAngle = Math.PI - robotDriveAngle - hubAngle;

                // Calculate limit if triangle is valid (otherwise no limit)
                // Basically, if robot can't rotate fast enough to keep up with the error caused
                // by
                // the initial velocity, we limit it.
                if (lookaheadAngle > 0) {
                    // Law of sines
                    double robotLookaheadDistance = robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
                    maxLinearVelocityMagnitude =
                            robotLookaheadDistance / LaunchCalculator.getTimeOfFlight(robotHubDistance);
                }

                // Apply limit to velocity
                if (initialLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
                    initialLinearVelocity =
                            initialLinearVelocity.times(maxLinearVelocityMagnitude / initialLinearVelocity.getNorm());
                }
            }

            // Apply chassis speeds
            double corScalar = MathUtil.clamp(
                    (Math.abs(parameters.driveAngle().minus(measuredRobotAngle).getDegrees())
                                    - driveLauncherCORMinErrorDeg.get())
                            / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                    0.0,
                    1.0);
            ChassisSpeeds fieldRelativeSpeedsWithOffset = GeomUtil.transformVelocity(
                    new ChassisSpeeds(initialLinearVelocity.getX(), initialLinearVelocity.getY(), omegaOutput),
                    LauncherConstants.LAUNCHER_TO_ROBOT.times(1.0 - corScalar),
                    measuredRobotAngle);

            drive.setRobotSpeeds(
                    ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeedsWithOffset, measuredRobotAngle));

            // Override robot setpoint speeds published by drive. We run our calculations
            // using the
            // speeds that will ultimately be applied once we are using the full
            // robot-to-launcher
            // transform. This prevents the setpoint from changing due to the shifting COR
            // of the
            // robot.
            ChassisSpeeds fieldRelativeSpeedsWithFullOffset = GeomUtil.transformVelocity(
                    new ChassisSpeeds(initialLinearVelocity.getX(), initialLinearVelocity.getY(), omegaOutput),
                    LauncherConstants.LAUNCHER_TO_ROBOT,
                    measuredRobotAngle);

            LaunchCalculator.getInstance()
                    .setDesiredFieldRelativeSpeedsOverride(ChassisSpeeds.discretize(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    fieldRelativeSpeedsWithFullOffset, measuredRobotAngle),
                            Constants.LOOP_PERIOD_SECONDS));

            // Log data
            Logger.recordOutput(
                    "LaunchCalculator/Driving/SetpointPose",
                    new Pose2d(robotPose.getTranslation(), parameters.driveAngle()));
            Logger.recordOutput("LaunchCalculator/Driving/AtGoalTolerance", isDriveAtLaunchGoal(drive));
            Logger.recordOutput(
                    "LaunchCalculator/Driving/ErrorPosition",
                    parameters.driveAngle().minus(measuredRobotAngle));
            Logger.recordOutput(
                    "LaunchCalculator/Driving/ErrorVelocityRadPerSec",
                    parameters.driveAngularVelocityRadPerSec() - measuredRobotRelativeSpeeds.omegaRadiansPerSecond);
            Logger.recordOutput("LaunchCalculator/Driving/MeasuredPosition", measuredRobotAngle);
            Logger.recordOutput(
                    "LaunchCalculator/Driving/MeasuredVelocityRadPerSec",
                    measuredRobotRelativeSpeeds.omegaRadiansPerSecond);
            Logger.recordOutput("LaunchCalculator/Driving/SetpointPosition", parameters.driveAngle());
            Logger.recordOutput(
                    "LaunchCalculator/Driving/SetpointVelocityRadPerSec", parameters.driveAngularVelocityRadPerSec());
        });
    }

    public static boolean isDriveAtLaunchGoal(Drive drive) {
        return MathUtil.isNear(
                LaunchCalculator.getInstance()
                        .getParameters(drive.getRobotPose(), drive.getRobotSpeeds())
                        .driveAngle()
                        .getRadians(),
                drive.getRobotPose().getRotation().getRadians(),
                Units.degreesToRadians(driveYawLaunchToleranceDeg.get()));
    }
}
