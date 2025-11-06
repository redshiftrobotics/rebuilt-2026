package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Camera.VisionPoseEstimation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final Camera[] cameras;

  private List<Consumer<VisionPoseEstimation>> timestampRobotPoseEstimateConsumers =
      new ArrayList<>();

  private boolean hasSuccessfulEstimation = false;

  public AprilTagVision(CameraIO... camerasIO) {
    this.cameras = Arrays.stream(camerasIO).map(io -> new Camera(io)).toArray(Camera[]::new);
  }

  @Override
  public void periodic() {

    // Run periodic for all cameras, as they are not real subsystems
    for (Camera camera : cameras) {
      camera.periodic();
    }

    List<Pose3d> robotPosesAccepted = new ArrayList<>();
    List<Pose3d> robotPosesRejected = new ArrayList<>();
    List<Pose3d> seenTagPoses = new ArrayList<>();
    List<Integer> seenTagIDs = new ArrayList<>();

    // Loop through all cameras
    for (Camera camera : cameras) {

      // Loop through all results that the camera has
      for (VisionPoseEstimation result : camera.getResults()) {

        (result.isSuccess() ? robotPosesAccepted : robotPosesRejected)
            .add(result.estimatedRobotPose());
        seenTagPoses.addAll(Arrays.asList(result.tagPositionsOnField()));
        seenTagIDs.addAll(Arrays.stream(result.tagsUsed()).boxed().toList());

        Logger.recordOutput("Vision/" + camera.getCameraName(), result);

        for (Consumer<VisionPoseEstimation> consumer : timestampRobotPoseEstimateConsumers) {
          consumer.accept(result);
        }
      }
    }

    hasSuccessfulEstimation = !robotPosesAccepted.isEmpty();

    Logger.recordOutput("Vision/robotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/robotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/seenTagPoses", seenTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/seenTagIDs", seenTagIDs.stream().mapToInt(Integer::intValue).toArray());
  }

  /** Get whether or not the vision system has a valid estimate */
  public boolean hasSuccessfulEstimate() {
    return hasSuccessfulEstimation;
  }

  /**
   * Set the last robot pose supplier for all cameras
   *
   * @param lastRobotPose the last robot pose supplier
   */
  public void setLastRobotPoseSupplier(Supplier<Pose2d> lastRobotPose) {
    for (Camera camera : cameras) {
      camera.setLastRobotPoseSupplier(lastRobotPose);
    }
  }

  /**
   * Add a consumer for the vision estimate
   *
   * @param timestampRobotPoseEstimateConsumer the consumer for the vision estimate
   */
  public void addVisionEstimateConsumer(
      Consumer<VisionPoseEstimation> timestampRobotPoseEstimateConsumer) {
    timestampRobotPoseEstimateConsumers.add(timestampRobotPoseEstimateConsumer);
  }

  @Override
  public String toString() {
    return String.format(
        "%s(%s)",
        getClass().getName(),
        Arrays.stream(cameras).map(Camera::getCameraName).collect(Collectors.joining(", ")));
  }
}
