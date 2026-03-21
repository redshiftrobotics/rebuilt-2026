// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.utility.MeanAngleFilter;
import frc.robot.utility.VirtualSubsystem;
import frc.robot.utility.geometry.AllianceMirrorUtil;
import frc.robot.utility.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator extends VirtualSubsystem {

    private static LaunchCalculator instance;

    private double hoodPositionOffset = 0.0;
    private double wheelRadPerSecOffset = 0.0;

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));
    private final MeanAngleFilter driveAngleFilter = new MeanAngleFilter((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));

    private double lastHoodAngle;
    private Rotation2d lastDriveAngle;

    public static LaunchCalculator getInstance() {
        if (instance == null) instance = new LaunchCalculator();
        return instance;
    }

    public record LaunchingParameters(
            boolean isValid,
            Rotation2d driveAngle,
            double driveAngularVelocityRadPerSec,
            double hoodPosition,
            double hoodVelocity,
            Double wheelRadPerSec,
            double distance,
            double distanceNoLookahead,
            double timeOfFlight,
            boolean passing) {}

    // Cache parameters
    private LaunchingParameters latestParameters = null;
    private ChassisSpeeds desiredFieldRelativeSpeedsOverride = new ChassisSpeeds();

    @Override
    public void periodic() {
        clearLaunchingParameters();
    }

    // Launching Maps
    private static final InterpolatingDoubleTreeMap hoodPositionMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap wheelRadPerSecMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {

        // Store offsets uses during match here
        final double wheelRadPerSecOffsetSetup = 0.0;
        final double hoodPositionOffsetSetup = 0.0;

        putTableDataTapeMeasure(8, 300 + wheelRadPerSecOffsetSetup + 35, 0.1 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(21, 325 + wheelRadPerSecOffsetSetup + 35, 0.1 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(35, 325 + wheelRadPerSecOffsetSetup + 35, 0.15 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(48, 370 + wheelRadPerSecOffsetSetup + 35, 0.15 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(62, 370 + wheelRadPerSecOffsetSetup + 35, 0.2 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(78, 400 + wheelRadPerSecOffsetSetup + 35, 0.2 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(90, 400 + wheelRadPerSecOffsetSetup + 35, 0.25 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(101, 400 + wheelRadPerSecOffsetSetup + 35, 0.3 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(112, 410 + wheelRadPerSecOffsetSetup + 35, 0.3 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(124, 425 + wheelRadPerSecOffsetSetup + 35, 0.35 + hoodPositionOffsetSetup);
        putTableDataTapeMeasure(139, 425 + wheelRadPerSecOffsetSetup + 35, 0.4 + hoodPositionOffsetSetup);

        timeOfFlightMap.put(3.60, 1.88 - 0.5);
        timeOfFlightMap.put(3.40, 1.57 - 0.5);
        timeOfFlightMap.put(3.15, 1.53 - 0.5);
        timeOfFlightMap.put(2.89, 1.44 - 0.5);
        timeOfFlightMap.put(1.49, 1.23 - 0.5);
        timeOfFlightMap.put(2.57, 1.48 - 0.5);
    }

    /**
     * Adds data to the launching maps based on tape measure readings from near edge of robot to near
     * edge of hub
     *
     * <p>Less ideal to use compared to getting distance from robot estimation due to potentially
     * mismatch of reality and robot estimation
     *
     * @param distanceInches edge-to-edge distance in inches from robot to hub, should be measured
     *     from near edge of robot bumper to near edge of hub
     * @param speedRadiansPerSecond outtake wheel speed in radians per second for the given distance
     * @param hoodPosition outtake hood position the given distance
     */
    public static void putTableDataTapeMeasure(
            double distanceInches, double speedRadiansPerSecond, double hoodPosition) {
        // Add half the hub width, half the bot width, and the launcher position
        // to convert from edge-to-edge to launcher-based distance to the goal
        double distanceMeters = Units.inchesToMeters(distanceInches)
                + FieldConstants.Hub.width / 2
                + DriveConstants.BUMPER_TO_BUMPER.getX() / 2
                - LauncherConstants.ROBOT_TO_LAUNCHER.getX();
        // By adding this value to the distance, the calculator will use smaller values
        // in the table and stop overshooting (it should be relative to launcher so we
        // subtract the launcher position)

        putTableData(distanceMeters, speedRadiansPerSecond, hoodPosition);
    }

    /**
     * Add data to the launching maps. Distance should be from the launcher to the goal.
     *
     * <p>It is recommended to let the robot calculate the distance in meters instead of measuring by
     * hand. Read the distance from "LauncherTuning/CalculatorDistance" when launcher is in Dashboard
     * Tuning mode
     *
     * @param distanceMeters
     * @param speedRadiansPerSecond outtake wheel speed in radians per second for the given distance
     * @param hoodPosition outtake hood position the given distance
     */
    public static void putTableData(double distanceMeters, double speedRadiansPerSecond, double hoodPosition) {
        hoodPositionMap.put(distanceMeters, hoodPosition);
        wheelRadPerSecMap.put(distanceMeters, speedRadiansPerSecond);
        // timeOfFlightMap.put(distanceMeters, null);
    }

    public static double getMinTimeOfFlight() {
        return getTimeOfFlight(LauncherConstants.MIN_DISTANCE);
    }

    public static double getMaxTimeOfFlight() {
        return getTimeOfFlight(LauncherConstants.MAX_DISTANCE);
    }

    public static Translation2d getTarget(Translation2d robotTranslation) {
        if (!isPassing(robotTranslation))
            return AllianceMirrorUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        else {
            double offset = (FieldConstants.fieldWidth / 4)
                    * (robotTranslation.getY() > FieldConstants.LinesHorizontal.center ? 1 : -1);
            return new Translation2d(
                    AllianceMirrorUtil.applyX(FieldConstants.LinesVertical.allianceZone / 2 + 1),
                    FieldConstants.LinesHorizontal.center + offset);
        }
    }

    public LaunchingParameters getParameters(Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity) {

        if (latestParameters != null) return latestParameters;

        boolean passing = isPassing(estimatedPose.getTranslation());
        Translation2d target = getTarget(estimatedPose.getTranslation());

        // Calculate estimated pose while accounting for phase delay
        estimatedPose = estimatedPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * LauncherConstants.PHASE_DELAY,
                robotRelativeVelocity.vyMetersPerSecond * LauncherConstants.PHASE_DELAY,
                robotRelativeVelocity.omegaRadiansPerSecond * LauncherConstants.PHASE_DELAY));

        Pose2d launcherPosition =
                estimatedPose.transformBy(GeomUtil.toTransform2d(LauncherConstants.ROBOT_TO_LAUNCHER));
        double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

        Rotation2d robotAngle = estimatedPose.getRotation();
        ChassisSpeeds robotVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(desiredFieldRelativeSpeedsOverride, robotAngle);

        // Calculate field relative launcher velocity
        ChassisSpeeds launcherVelocity = GeomUtil.transformVelocity(
                robotVelocity,
                LauncherConstants.ROBOT_TO_LAUNCHER.getTranslation().toTranslation2d(),
                robotAngle);

        // Account for imparted velocity by robot (launcher) to offset
        double timeOfFlight = getTimeOfFlight(launcherToTargetDistance);
        Pose2d lookaheadPose = launcherPosition;
        double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

        for (int i = 0; i < 20; i++) { // number of iterations to converge, should be more than enough
            timeOfFlight = getTimeOfFlight(lookaheadLauncherToTargetDistance);
            double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
            double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
            lookaheadPose = new Pose2d(
                    launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    launcherPosition.getRotation());
            lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Account for launcher being off center
        Pose2d lookaheadRobotPose = lookaheadPose.transformBy(
                GeomUtil.toTransform2d(LauncherConstants.ROBOT_TO_LAUNCHER).inverse());
        Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

        // Calculate interpolated values from maps
        double wheelRadPerSec = wheelRadPerSecMap.get(lookaheadLauncherToTargetDistance) + wheelRadPerSecOffset;
        double hoodPosition = hoodPositionMap.get(lookaheadLauncherToTargetDistance) + hoodPositionOffset;

        // Calculate average hood velocity
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodPosition;
        double hoodVelocity = hoodAngleFilter.calculate((hoodPosition - lastHoodAngle) / Constants.LOOP_PERIOD_SECONDS);
        lastHoodAngle = hoodPosition;

        // Calculate average drive angular velocity
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        Rotation2d driveAngularVelocity =
                driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).div(Constants.LOOP_PERIOD_SECONDS));
        lastDriveAngle = driveAngle;

        // Constructor parameters
        latestParameters = new LaunchingParameters(
                lookaheadLauncherToTargetDistance <= LauncherConstants.MAX_DISTANCE
                        && lookaheadLauncherToTargetDistance >= LauncherConstants.MIN_DISTANCE,
                driveAngle,
                driveAngularVelocity.getRadians(),
                hoodPosition,
                hoodVelocity,
                wheelRadPerSec,
                lookaheadLauncherToTargetDistance,
                launcherToTargetDistance,
                timeOfFlight,
                passing);

        // Log calculated values
        Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
        Logger.recordOutput("LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

        Logger.recordOutput("LaunchCalculator/latestParameters", latestParameters);

        Logger.recordOutput("LaunchCalculator/HoodPositionOffset", hoodPositionOffset);
        Logger.recordOutput("LaunchCalculator/WheelRadPerSecOffset", wheelRadPerSecOffset);

        return latestParameters;
    }

    private static Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
        Rotation2d hubAngle = new Rotation2d(Math.asin(MathUtil.clamp(
                LauncherConstants.ROBOT_TO_LAUNCHER.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                -1.0,
                1.0)));
        Rotation2d driveAngle = fieldToHubAngle
                .plus(hubAngle)
                .plus(LauncherConstants.ROBOT_TO_LAUNCHER.getRotation().toRotation2d());
        return driveAngle;
    }

    private static boolean isPassing(Translation2d robotTranslation) {
        return AllianceMirrorUtil.applyX(robotTranslation.getX()) > FieldConstants.LinesVertical.hubCenter;
    }

    public static double getTimeOfFlight(double distance) {
        Double value = timeOfFlightMap.get(distance);
        if (value == null) {
            Rotation2d angle = HoodIO.getAngle(hoodPositionMap.get(distance));
            return MathematicalShotCalculator.timeOfFlight(
                    angle, MathematicalShotCalculator.calculateVelocity(distance, angle), distance);
        }
        return value;
    }

    public void clearLaunchingParameters() {
        latestParameters = null;
    }

    /**
     * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
     *
     * @param robotTranslation The translation of the center of the robot.
     * @return The target pose for the aimed robot.
     */
    public static Pose2d getStationaryAimedPose(Translation2d robotTranslation) {
        return new Pose2d(
                robotTranslation,
                getDriveAngleWithLauncherOffset(GeomUtil.toPose2d(robotTranslation), getTarget(robotTranslation)));
    }

    /** Adjusts the hood angle offset up or down the specified amount. */
    public void incrementHoodPosition(double delta) {
        hoodPositionOffset += delta;
    }

    public double getHoodPositionOffset() {
        return hoodPositionOffset;
    }

    /** Adjusts the wheel speed offset up or down the specified amount. */
    public void incrementWheelRadPerSec(double delta) {
        wheelRadPerSecOffset += delta;
    }

    public double getWheelRadPerSecOffset() {
        return wheelRadPerSecOffset;
    }

    public void resetOffsets() {
        hoodPositionOffset = 0.0;
        wheelRadPerSecOffset = 0.0;
    }

    public void setDesiredFieldRelativeSpeedsOverride(ChassisSpeeds desiredFieldRelativeSpeedsOverride) {
        this.desiredFieldRelativeSpeedsOverride = desiredFieldRelativeSpeedsOverride;
    }
}
