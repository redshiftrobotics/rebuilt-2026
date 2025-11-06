package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumberGroup;
import java.util.Arrays;
import java.util.DoubleSummaryStatistics;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

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

  private Set<Integer> tagsIdsOnField;

  private VisionPoseEstimation[] results = new VisionPoseEstimation[0];

  private Supplier<Pose2d> lastRobotPoseSupplier;

  private final Alert missingCameraAlert;

  public static record VisionPoseEstimation(
      boolean hasNewData,
      Pose3d estimatedRobotPose,
      double timestampSecondFPGA,
      int[] tagsUsed,
      Pose3d[] tagPositionsOnField,
      VisionResultStatus status,
      Matrix<N3, N1> standardDeviations) {
    public boolean isSuccess() {
      return status.isSuccess();
    }
  }

  /**
   * Create a new robot camera with IO layer
   *
   * @param io camera implantation
   */
  public Camera(CameraIO io) {
    this.io = io;

    io.setAprilTagFieldLayout(VisionConstants.FIELD);
    this.tagsIdsOnField =
        VisionConstants.FIELD.getTags().stream().map((tag) -> tag.ID).collect(Collectors.toSet());

    this.missingCameraAlert =
        new Alert(String.format("Missing cameras %s", getCameraName()), Alert.AlertType.kWarning);
  }

  /** Get name of camera as specified by IO */
  public String getCameraName() {
    return io.getCameraPosition() + " (" + io.getCameraName() + ")";
  }

  /** Run periodic of module. Updates the set of loggable inputs, updating vision result. */
  public void periodic() {
    Logger.processInputs("Vision/" + getCameraName(), inputs);
    io.updateInputs(inputs);
    missingCameraAlert.set(!inputs.connected);

    results = new VisionPoseEstimation[inputs.updatesReceived];
    for (int i = 0; i < inputs.updatesReceived; i++) {
      Pose3d[] tagPositionsOnField = getTagPositionsOnField(inputs.tagsUsed[i]);

      results[i] =
          new VisionPoseEstimation(
              inputs.hasNewData[i],
              inputs.estimatedRobotPose[i],
              inputs.timestampSecondFPGA[i],
              inputs.tagsUsed[i],
              tagPositionsOnField,
    }
  }

  public void setLastRobotPoseSupplier(Supplier<Pose2d> lastRobotPose) {
    this.lastRobotPoseSupplier = lastRobotPose;
  }

  public VisionPoseEstimation[] getResults() {
    return results;
  }

  private Pose3d[] getTagPositionsOnField(int[] tagsUsed) {
    return Arrays.stream(tagsUsed)
        .mapToObj(VisionConstants.FIELD::getTagPose)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  /**
   * Get standard deviations of the vision measurements. Higher values numbers here means trust
   * global measurements from this camera less. The matrix is in the form [x, y, theta], with units
   * in meters and radians.
   */
  private Matrix<N3, N1> getStandardDeviations(Pose3d[] tagPositionsOnField, Pose3d lastRobotPose) {

    // Get data about distance to each tag that is present on field
    DoubleSummaryStatistics distancesToTags =
        Arrays.stream(tagPositionsOnField)
            .mapToDouble(
                (tagPose3d) ->
                    tagPose3d.getTranslation().getDistance(lastRobotPose.getTranslation()))
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

  private VisionResultStatus getStatus(VisionPoseEstimation estimation) {

    if (!estimation.hasNewData) {
      return VisionResultStatus.NO_DATA;
    }

    if (estimation.tagsUsed.length == 0) {
      return VisionResultStatus.NO_TARGETS_VISIBLE;
    }

    if (!Arrays.stream(estimation.tagsUsed).allMatch(tagsIdsOnField::contains)) {
      return VisionResultStatus.INVALID_TAG;
    }

    if (estimation.estimatedRobotPose.getX() < 0
        || estimation.estimatedRobotPose.getY() < 0
        || estimation.estimatedRobotPose.getX() > VisionConstants.FIELD.getFieldLength()
        || estimation.estimatedRobotPose.getY() > VisionConstants.FIELD.getFieldWidth()) {
      return VisionResultStatus.INVALID_POSE_OUTSIDE_FIELD;
    }

    if (!MathUtil.isNear(0, estimation.estimatedRobotPose.getZ(), zHeightToleranceMeters.get())) {
      return VisionResultStatus.Z_HEIGHT_BAD;
    }

    double pitchAndRollToleranceValueRadians =
        Units.degreesToRadians(pitchAndRollToleranceDegrees.get());
    if (!MathUtil.isNear(
            0,
            estimation.estimatedRobotPose.getRotation().getX(),
            pitchAndRollToleranceValueRadians)
        && !MathUtil.isNear(
            0,
            estimation.estimatedRobotPose.getRotation().getY(),
            pitchAndRollToleranceValueRadians)) {
      return VisionResultStatus.PITCH_OR_ROLL_BAD;
    }

    if (lastRobotPoseSupplier != null) {
      Pose2d estimatedRobotPose2d = estimation.estimatedRobotPose.toPose2d();
      Pose2d lastRobotPose = lastRobotPoseSupplier.get();

      if (!MathUtil.isNear(
          estimatedRobotPose2d.getRotation().getDegrees(),
          lastRobotPose.getRotation().getDegrees(),
          maxValidDistanceAwayFromCurrentHeadingDegrees.get())) {
        return VisionResultStatus.NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION;
      }

      if (estimatedRobotPose2d.getTranslation().getDistance(lastRobotPose.getTranslation())
          > maxValidDistanceAwayFromCurrentEstimateMeters.get()) {
        return VisionResultStatus.TOO_FAR_FROM_EXISTING_ESTIMATE;
      }
    }

    return VisionResultStatus.SUCCESSFUL;
  }

  public enum VisionResultStatus {
    NO_DATA(false),

    NO_TARGETS_VISIBLE(false),
    INVALID_TAG(false),

    INVALID_POSE_OUTSIDE_FIELD(false),
    Z_HEIGHT_BAD(false),
    PITCH_OR_ROLL_BAD(false),

    NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION(false),
    TOO_FAR_FROM_EXISTING_ESTIMATE(false),

    SUCCESSFUL(true);

    public final boolean success;

    private VisionResultStatus(boolean success) {
      this.success = success;
    }

    public boolean isSuccess() {
      return success;
    }
  }
}
