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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.MeanAngleFilter;
import frc.robot.utility.VirtualSubsystem;
import frc.robot.utility.geometry.AllianceMirrorUtil;
import frc.robot.utility.geometry.GeomUtil;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumberGroup;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator extends VirtualSubsystem {
  public static final Transform3d robotToLauncher =
      new Transform3d(
          Units.inchesToMeters(-9.937105),
          0.0,
          Units.inchesToMeters(17.731846 + (4.0 / 2.0)),
          Rotation3d.kZero); // From CAD

  private static final Translation2d launcherToRobot =
      robotToLauncher.getTranslation().toTranslation2d().unaryMinus();

  private static LaunchCalculator instance;

  private double hoodPositionOffset = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));
  private final MeanAngleFilter driveAngleFilter =
      new MeanAngleFilter((int) (0.1 / Constants.LOOP_PERIOD_SECONDS));

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

  public static final InterpolatingDoubleTreeMap hoodAngleMap = null;
  // Launching Maps
  private static final InterpolatingDoubleTreeMap hoodPositionMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap wheelRadPerSecMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = null;
  // new InterpolatingDoubleTreeMap();

  private static final double minDistance = 0.9;
  private static final double maxDistance = 4.9;
  private static final double phaseDelay = 0.03; // estimate

  static {
    putTableData(8, 300, 0.1);
    putTableData(21, 325, 0.1);
    putTableData(35, 325, 0.15);
    putTableData(48, 370, 0.15);
    putTableData(62, 370, 0.2);
    putTableData(78, 400, 0.2);
    putTableData(90, 400, 0.25);
    putTableData(101, 400, 0.3);
    putTableData(112, 410, 0.3);
    putTableData(124, 425, 0.35);
    putTableData(139, 425, 0.4);
  }

  public static void putTableData(
      double distanceInches, double speedRadiansPerSecond, double hoodPosition) {
    double distanceMeters = Units.inchesToMeters(distanceInches);

    hoodPositionMap.put(distanceMeters, hoodPosition);
    wheelRadPerSecMap.put(distanceMeters, speedRadiansPerSecond);
    // timeOfFlightMap.put(distanceMeters, null);
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public static Translation2d getTarget() {
    return AllianceMirrorUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
  }

  public LaunchingParameters getParameters(
      Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity) {

    boolean passing =
        AllianceMirrorUtil.applyX(estimatedPose.getX()) > FieldConstants.LinesVertical.hubCenter;

    Translation2d target = getTarget();

    // Calculate estimated pose while accounting for phase delay
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Pose2d launcherPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(robotToLauncher));
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    Rotation2d robotAngle = estimatedPose.getRotation();
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(desiredFieldRelativeSpeedsOverride, robotAngle);

    // Calculate field relative launcher velocity
    ChassisSpeeds launcherVelocity =
        GeomUtil.transformVelocity(
            robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) { // number of iterations to converge, should be more than enough
      timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(GeomUtil.toTransform2d(robotToLauncher).inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate interpolated values from maps
    double wheelRadPerSec = wheelRadPerSecMap.get(lookaheadLauncherToTargetDistance);
    double hoodPosition = hoodPositionMap.get(lookaheadLauncherToTargetDistance);

    // Calculate average hood velocity
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodPosition;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodPosition - lastHoodAngle) / Constants.LOOP_PERIOD_SECONDS);
    lastHoodAngle = hoodPosition;

    // Calculate average drive angular velocity
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    Rotation2d driveAngularVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).div(Constants.LOOP_PERIOD_SECONDS));
    lastDriveAngle = driveAngle;

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            lookaheadLauncherToTargetDistance >= minDistance
                && lookaheadLauncherToTargetDistance <= maxDistance,
            driveAngle,
            driveAngularVelocity.getRadians(),
            hoodPosition + hoodPositionOffset,
            hoodVelocity,
            wheelRadPerSec,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

    Logger.recordOutput("LaunchCalculator/latestParameters", latestParameters);

    return latestParameters;
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    if (timeOfFlightMap == null) {
      Rotation2d angle = HoodIO.getAngle(hoodPositionMap.get(distance));
      return MathematicalShotCalculator.timeOfFlight(
          angle, MathematicalShotCalculator.calculateVelocity(distance, angle), distance);
    }
    return timeOfFlightMap.get(distance);
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
        getDriveAngleWithLauncherOffset(GeomUtil.toPose2d(robotTranslation), getTarget()));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodPosition(double delta) {
    hoodPositionOffset += delta;
  }

  public double getHoodPositionOffset() {
    return hoodPositionOffset;
  }

  private static final TunableNumberGroup launchingGroup =
      new TunableNumberGroup("LaunchCalculator/Driving");

  private static final TunableNumber driveLaunchKp = launchingGroup.number("kP", 8.0);
  private static final TunableNumber driveLaunchKd = launchingGroup.number("kD", 0.5);
  private static final TunableNumber driveControllerYawToleranceDeg =
      launchingGroup.number("ControllerYawToleranceDeg", 1.0);

  private static final TunableNumber driveYawLaunchToleranceDeg =
      launchingGroup.number("YawToleranceDeg", 5.0);
  private static final TunableNumber drivePitchLaunchToleranceDeg =
      launchingGroup.number("PitchToleranceDeg", 5.0);
  private static final TunableNumber driveRollLaunchToleranceDeg =
      launchingGroup.number("RollToleranceDeg", 5.0);
  private static final TunableNumber driveLaunchMaxPolarVelocityRadPerSec =
      launchingGroup.number("MaxPolarVelocityRadPerSec", 0.6);
  private static final TunableNumber driveLauncherCORMinErrorDeg =
      launchingGroup.number("DriveLauncherCORMinErrorDeg", 15.0);
  private static final TunableNumber driveLauncherCORMaxErrorDeg =
      launchingGroup.number("DriveLauncherCORMaxErrorDeg", 30.0);

  public static Command driveWhileLaunching(
      Drive drive, final Supplier<ChassisSpeeds> initialRobotRelativeSpeedsSupplier) {
    return drive.run(
        () -> {
          final Pose2d robotPose = drive.getRobotPose();
          final ChassisSpeeds measuredRobotRelativeSpeeds = drive.getRobotSpeeds();
          final Rotation2d measuredRobotAngle = robotPose.getRotation();

          // Run PID controller
          final LaunchingParameters parameters =
              LaunchCalculator.getInstance().getParameters(robotPose, measuredRobotRelativeSpeeds);

          double omegaOutput =
              parameters.driveAngularVelocityRadPerSec()
                  + (parameters.driveAngle().minus(measuredRobotAngle).getRadians()
                      * driveLaunchKp.get())
                  + ((parameters.driveAngularVelocityRadPerSec()
                          - measuredRobotRelativeSpeeds.omegaRadiansPerSecond)
                      * driveLaunchKd.get());

          // within tolerance, no need to drive
          if (MathUtil.isNear(
              parameters.driveAngle().getDegrees(),
              measuredRobotAngle.getDegrees(),
              driveControllerYawToleranceDeg.get())) {
            omegaOutput = 0.0;
          }

          ChassisSpeeds initialFieldRelativeSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  initialRobotRelativeSpeedsSupplier.get(),
                  AllianceMirrorUtil.apply(measuredRobotAngle));
          Translation2d initialLinearVelocity =
              new Translation2d(
                  initialFieldRelativeSpeeds.vxMetersPerSecond,
                  initialFieldRelativeSpeeds.vyMetersPerSecond);

          // Only limit if launching, not passing
          if (!parameters.passing()) {
            // Calculate max linear velocity magnitude based on the max polar velocity
            // Basically, if the robot is moving (linearly) faster than it can rotate
            // to correct its angle to the hub, we cap the velocity so that it can always face the
            // hub
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            double robotDriveAngle =
                Math.abs(
                    AllianceMirrorUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(robotPose.getTranslation())
                        .getAngle()
                        .minus(initialLinearVelocity.getAngle())
                        .getRadians());
            double robotHubDistance = parameters.distanceNoLookahead();
            double hubAngle =
                driveLaunchMaxPolarVelocityRadPerSec.get()
                    * LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
            double lookaheadAngle = Math.PI - robotDriveAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            // Basically, if robot can't rotate fast enough to keep up with the error caused by
            // the initial velocity, we limit it.
            if (lookaheadAngle > 0) {
              // Law of sines
              double robotLookaheadDistance =
                  robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
              maxLinearVelocityMagnitude =
                  robotLookaheadDistance
                      / LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
            }

            // Apply limit to velocity
            if (initialLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
              initialLinearVelocity =
                  initialLinearVelocity.times(
                      maxLinearVelocityMagnitude / initialLinearVelocity.getNorm());
            }
          }

          // Apply chassis speeds
          double corScalar =
              MathUtil.clamp(
                  (Math.abs(parameters.driveAngle().minus(measuredRobotAngle).getDegrees())
                          - driveLauncherCORMinErrorDeg.get())
                      / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                  0.0,
                  1.0);
          ChassisSpeeds fieldRelativeSpeedsWithOffset =
              GeomUtil.transformVelocity(
                  new ChassisSpeeds(
                      initialLinearVelocity.getX(), initialLinearVelocity.getY(), omegaOutput),
                  launcherToRobot.times(1.0 - corScalar),
                  measuredRobotAngle);

          drive.setRobotSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeSpeedsWithOffset, measuredRobotAngle));

          // Override robot setpoint speeds published by drive. We run our calculations using the
          // speeds that will ultimately be applied once we are using the full robot-to-launcher
          // transform. This prevents the setpoint from changing due to the shifting COR of the
          // robot.
          ChassisSpeeds fieldRelativeSpeedsWithFullOffset =
              GeomUtil.transformVelocity(
                  new ChassisSpeeds(
                      initialLinearVelocity.getX(), initialLinearVelocity.getY(), omegaOutput),
                  launcherToRobot,
                  measuredRobotAngle);

          LaunchCalculator.getInstance()
              .setDesiredFieldRelativeSpeedsOverride(
                  ChassisSpeeds.discretize(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          fieldRelativeSpeedsWithFullOffset, measuredRobotAngle),
                      Constants.LOOP_PERIOD_SECONDS));

          // Log data
          Logger.recordOutput(
              "LaunchCalculator/Driving/SetpointPose",
              new Pose2d(robotPose.getTranslation(), parameters.driveAngle()));
          Logger.recordOutput(
              "LaunchCalculator/Driving/AtGoalTolerance", isDriveAtLaunchGoal(drive));
          Logger.recordOutput(
              "LaunchCalculator/Driving/ErrorPosition",
              parameters.driveAngle().minus(measuredRobotAngle));
          Logger.recordOutput(
              "LaunchCalculator/Driving/ErrorVelocityRadPerSec",
              parameters.driveAngularVelocityRadPerSec()
                  - measuredRobotRelativeSpeeds.omegaRadiansPerSecond);
          Logger.recordOutput("LaunchCalculator/Driving/MeasuredPosition", measuredRobotAngle);
          Logger.recordOutput(
              "LaunchCalculator/Driving/MeasuredVelocityRadPerSec",
              measuredRobotRelativeSpeeds.omegaRadiansPerSecond);
          Logger.recordOutput("LaunchCalculator/Driving/SetpointPosition", parameters.driveAngle());
          Logger.recordOutput(
              "LaunchCalculator/Driving/SetpointVelocityRadPerSec",
              parameters.driveAngularVelocityRadPerSec());
        });
  }

  public static boolean isDriveAtLaunchGoal(Drive drive) {
    Rotation3d rotation3d = drive.getRawGyroRotation3d();
    boolean inPitchAndRollTolerance =
        (Math.abs(rotation3d.getX()) <= Units.degreesToRadians(driveRollLaunchToleranceDeg.get())
            && Math.abs(rotation3d.getY())
                <= Units.degreesToRadians(drivePitchLaunchToleranceDeg.get()));

    return DriverStation.isEnabled()
        && Math.abs(
                drive
                    .getRobotPose()
                    .getRotation()
                    .minus(
                        LaunchCalculator.getInstance()
                            .getParameters(drive.getRobotPose(), drive.getRobotSpeeds())
                            .driveAngle())
                    .getRadians())
            <= Units.degreesToRadians(driveYawLaunchToleranceDeg.get())
        && inPitchAndRollTolerance;
  }

  public void setDesiredFieldRelativeSpeedsOverride(
      ChassisSpeeds desiredFieldRelativeSpeedsOverride) {
    this.desiredFieldRelativeSpeedsOverride = desiredFieldRelativeSpeedsOverride;
  }
}
