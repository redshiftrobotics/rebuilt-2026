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
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator extends VirtualSubsystem {
  public static final Transform3d robotToLauncher =
      new Transform3d(
          Units.inchesToMeters(-9.937105),
          0.0,
          Units.inchesToMeters(17.731846 + (4.0 / 2.0)),
          Rotation3d.kZero); // From CAD

  private static LaunchCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;

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
      Rotation2d hoodAngle,
      double hoodVelocityRadPerSec,
      AngularVelocity flywheelSpeed,
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
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final double minDistance = 0.9;
  private static final double maxDistance = 4.9;
  private static final double phaseDelay = 0.03; // estimate

  static {

    // Distances are from LauncherToTargetDistance
    hoodAngleMap.put(0.96, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(1.16, Rotation2d.fromDegrees(12.0));
    hoodAngleMap.put(1.58, Rotation2d.fromDegrees(14.0));
    hoodAngleMap.put(2.07, Rotation2d.fromDegrees(18.5));
    hoodAngleMap.put(2.37, Rotation2d.fromDegrees(22.0));
    hoodAngleMap.put(2.47, Rotation2d.fromDegrees(23.0));
    hoodAngleMap.put(2.70, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.94, Rotation2d.fromDegrees(25.0));
    hoodAngleMap.put(3.48, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.92, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(4.35, Rotation2d.fromDegrees(34.0));
    hoodAngleMap.put(4.84, Rotation2d.fromDegrees(38.0));

    // radians per second
    flywheelSpeedMap.put(0.96, 150.0);
    flywheelSpeedMap.put(1.16, 155.0);
    flywheelSpeedMap.put(1.58, 160.0);
    flywheelSpeedMap.put(2.07, 165.0);
    flywheelSpeedMap.put(2.37, 170.0);
    flywheelSpeedMap.put(2.47, 170.0);
    flywheelSpeedMap.put(2.70, 170.0);
    flywheelSpeedMap.put(2.94, 175.0);
    flywheelSpeedMap.put(3.48, 175.0);
    flywheelSpeedMap.put(3.92, 180.0);
    flywheelSpeedMap.put(4.35, 185.0);
    flywheelSpeedMap.put(4.84, 190.0);

    // seconds
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
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
    double hoodAngle = hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
    double flywheelVelocity = flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Calculate average hood velocity
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_PERIOD_SECONDS);
    lastHoodAngle = hoodAngle;

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
            Rotation2d.fromDegrees(hoodAngle + Units.degreesToRadians(hoodAngleOffsetDeg)),
            hoodVelocity,
            RadiansPerSecond.of(flywheelVelocity),
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
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }

  public double getHoodAngleOffsetDeg() {
    return hoodAngleOffsetDeg;
  }

  private static final TunableNumberGroup launchingGroup =
      new TunableNumberGroup("LaunchCalculator/Driving");

  private static final TunableNumber driveLaunchKp = launchingGroup.number("kP", 8.0);
  private static final TunableNumber driveLaunchKd = launchingGroup.number("kD", 0.5);
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
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            double robotAngle =
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
            double lookaheadAngle = Math.PI - robotAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            if (lookaheadAngle > 0) {
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
          Translation2d launcherToRobot =
              robotToLauncher.getTranslation().toTranslation2d().unaryMinus();
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
