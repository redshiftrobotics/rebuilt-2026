package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.vision.VisionConstants.CameraPosition;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumberGroup;
import java.util.ArrayList;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Wrapper for CameraIO layer */
public class Camera {

  private static final TunableNumberGroup group = new TunableNumberGroup("VisionResultsStatus");

  private static final TunableNumber xyStdDevCoefficient =
      group.number("xyStdDevCoefficient", 0.075);
  private static final TunableNumber thetaStdDevCoefficient =
      group.number("thetaStdDevCoefficient", 0.085);

  private static final TunableNumber zHeightToleranceMeters =
      group.number("zHeightToleranceMeters", 0.6);
  private static final TunableNumber pitchAndRollToleranceDegrees =
      group.number("pitchToleranceDegrees", 10.0);

  private static final TunableNumber maxValidDistanceAwayFromCurrentEstimateMeters =
      group.number("maxValidDistanceFromCurrentEstimateMeters", 10.0);
  private static final TunableNumber maxValidDistanceAwayFromCurrentHeadingDegrees =
      group.number("gyroFilteringToleranceDegrees", 30.0);

  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

  private List<ProcessedEstimatedRobotPose> results = List.of();

  private final Alert missingCameraAlert;

  private final Supplier<Pose2d> robotPoseSupplier;

  private AprilTagFieldLayout aprilTagFieldLayout = null;

  private boolean filterBasedOnLastPose = false;
  private boolean filterBasedOnGyro = false;

  public static record ProcessedEstimatedRobotPose(
      Pose3d estimatedPose,
      double timestampSeconds,
      VisionResultStatus status,
      Matrix<N3, N1> standardDeviations,
      List<Pose3d> tagPositionsOnField) {}

  /**
   * Create a new robot camera with IO layer
   *
   * @param io camera implantation
   */
  public Camera(CameraIO io, Supplier<Pose2d> robotPoseSupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;

    this.missingCameraAlert =
        new Alert(String.format("Missing cameras %s", getCameraName()), Alert.AlertType.kWarning);
  }

  /** Set april tag field layout to use */
  public void setAprilTagFieldLayout(AprilTagFieldLayout layout) {
    io.setAprilTagFieldLayout(layout);
    this.aprilTagFieldLayout = layout;
  }

  public String getCameraName() {
    return io.getCameraName();
  }

  public CameraPosition getCameraPosition() {
    return io.getCameraPosition();
  }

  /** Run periodic of module. Updates the set of loggable inputs, updating vision result. */
  public void periodic() {
    io.setLastRobotPose(robotPoseSupplier.get());

    Logger.processInputs("Vision/" + getCameraPosition(), inputs);
    io.updateInputs(inputs);

    missingCameraAlert.set(!inputs.connected);

    results = io.getEstimates().stream().map(this::processVision).toList();
  }

  public List<ProcessedEstimatedRobotPose> getResults() {
    return results;
  }

  private ProcessedEstimatedRobotPose processVision(EstimatedRobotPose estimate) {

    VisionResultStatus status = VisionResultStatus.NO_DATA;

    Matrix<N3, N1> standardDeviations =
        VecBuilder.fill(
            Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    final List<Pose3d> tagPositionsOnField = new ArrayList<>();

    if (aprilTagFieldLayout == null) {
      status = VisionResultStatus.NO_FIELD_LAYOUT;
    } else {
      status =
          getStatus(
              estimate.estimatedPose,
              estimate.targetsUsed,
              estimate.strategy,
              robotPoseSupplier.get());

      for (PhotonTrackedTarget target : estimate.targetsUsed) {
        Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        tagPoseOptional.ifPresent(tagPositionsOnField::add);
      }
    }

    if (status.isSuccess()) {
      standardDeviations = getStandardDeviations(tagPositionsOnField, robotPoseSupplier.get());
    }

    return new ProcessedEstimatedRobotPose(
        estimate.estimatedPose,
        estimate.timestampSeconds,
        status,
        standardDeviations,
        tagPositionsOnField);
  }

  /**
   * Get standard deviations of the vision measurements. Higher values numbers here means trust
   * global measurements from this camera less. The matrix is in the form [x, y, theta], with units
   * in meters and radians.
   */
  private Matrix<N3, N1> getStandardDeviations(List<Pose3d> tagPoses, Pose2d lastRobotPose) {

    Pose3d lastRobotPose3d = new Pose3d(lastRobotPose);

    // Get data about distance to each tag that is present on field
    DoubleSummaryStatistics distancesToTags =
        tagPoses.stream()
            .mapToDouble(
                (estimatedPose) ->
                    estimatedPose.getTranslation().getDistance(lastRobotPose3d.getTranslation()))
            .summaryStatistics();

    // This equation is heuristic, good enough but can probably be improved
    // Larger distances to tags and fewer observed tags result in higher uncertainty (larger
    // standard deviations). Average distance increases uncertainty exponentially while more
    // tags decreases uncertainty linearly
    double standardDeviation =
        distancesToTags.getCount() > 0
            ? Math.pow(distancesToTags.getAverage(), 2) * Math.pow(distancesToTags.getCount(), -1)
            : Double.POSITIVE_INFINITY;

    double xyStandardDeviation = xyStdDevCoefficient.get() * standardDeviation;

    double thetaStandardDeviation = thetaStdDevCoefficient.get() * standardDeviation;

    // x, y, theta
    return VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation);
  }

  private VisionResultStatus getStatus(
      Pose3d estimatedRobotPose,
      List<PhotonTrackedTarget> usedTargets,
      PoseStrategy poseStrategy,
      Pose2d lastRobotPose) {

    if (usedTargets.isEmpty()) {
      return VisionResultStatus.NO_TARGETS_VISIBLE;
    }

    if (!usedTargets.stream()
        .allMatch(tag -> aprilTagFieldLayout.getTagPose(tag.getFiducialId()).isPresent())) {
      return VisionResultStatus.INVALID_TAG;
    }

    if (estimatedRobotPose.getX() < 0
        || estimatedRobotPose.getY() < 0
        || estimatedRobotPose.getX() > aprilTagFieldLayout.getFieldLength()
        || estimatedRobotPose.getY() > aprilTagFieldLayout.getFieldWidth()) {
      return VisionResultStatus.INVALID_POSE_OUTSIDE_FIELD;
    }

    if (!MathUtil.isNear(0, estimatedRobotPose.getZ(), zHeightToleranceMeters.get())) {
      return VisionResultStatus.Z_HEIGHT_BAD;
    }

    double pitchAndRollToleranceValueRadians =
        Units.degreesToRadians(pitchAndRollToleranceDegrees.get());
    if (!MathUtil.isNear(
            0, estimatedRobotPose.getRotation().getX(), pitchAndRollToleranceValueRadians)
        && !MathUtil.isNear(
            0, estimatedRobotPose.getRotation().getY(), pitchAndRollToleranceValueRadians)) {
      return VisionResultStatus.PITCH_OR_ROLL_BAD;
    }

    if (filterBasedOnGyro
        && !MathUtil.isNear(
            estimatedRobotPose
                .getRotation()
                .toRotation2d()
                .minus(lastRobotPose.getRotation())
                .getDegrees(),
            0,
            maxValidDistanceAwayFromCurrentHeadingDegrees.get())) {
      return VisionResultStatus.NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION;
    }

    if (filterBasedOnLastPose
        && estimatedRobotPose
                .toPose2d()
                .getTranslation()
                .getDistance(lastRobotPose.getTranslation())
            > maxValidDistanceAwayFromCurrentEstimateMeters.get()) {
      return VisionResultStatus.TOO_FAR_FROM_EXISTING_ESTIMATE;
    }

    return poseStrategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        ? VisionResultStatus.MULTI_TAG_SUCCESSFUL
        : VisionResultStatus.SINGLE_TAG_SUCCESSFUL;
  }

  public enum VisionResultStatus {
    NO_DATA(false),
    NO_FIELD_LAYOUT(false),

    NO_TARGETS_VISIBLE(false),
    INVALID_TAG(false),

    INVALID_POSE_OUTSIDE_FIELD(false),
    Z_HEIGHT_BAD(false),
    PITCH_OR_ROLL_BAD(false),

    NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION(false),
    TOO_FAR_FROM_EXISTING_ESTIMATE(false),

    MULTI_TAG_SUCCESSFUL(true),
    SINGLE_TAG_SUCCESSFUL(true);

    public final boolean success;

    private VisionResultStatus(boolean success) {
      this.success = success;
    }

    public boolean isSuccess() {
      return success;
    }
  }
}
