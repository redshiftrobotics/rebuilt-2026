package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionConstants.CameraPosition;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraIOPhotonVision implements CameraIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;

  private final CameraPosition cameraPosition;

  public CameraIOPhotonVision(CameraConfig config) {
    this.cameraPosition = config.cameraPosition();

    // --- Setup Camera ---
    camera = new PhotonCamera(config.cameraName());

    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);

    // --- Setup Pose Estimator ---

    // MULTI_TAG_PNP_ON_COPROCESSOR:
    // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#enabling-multitag

    // PhotonPoseEstimator:
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#using-a-photonposeestimator

    photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  protected PhotonCamera getCamera() {
    return camera;
  }

  protected PhotonPoseEstimator getPhotonPoseEstimator() {
    return photonPoseEstimator;
  }

  @Override
  public CameraPosition getCameraPosition() {
    return cameraPosition;
  }

  @Override
  public void setAprilTagFieldLayout(AprilTagFieldLayout fieldTags) {
    photonPoseEstimator.setFieldTags(fieldTags);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {

    List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

    List

    for (int i = 0; i < pipelineResults.size(); i++) {
      PhotonPipelineResult result = pipelineResults.get(i);
      Optional<EstimatedRobotPose> estimatedRobotPoseOptional = photonPoseEstimator.update(result);

      if (estimatedRobotPoseOptional.isPresent()) {

        EstimatedRobotPose estimateRobotPose = estimatedRobotPoseOptional.get();
        inputs.poseEstimates[i] = estimateRobotPose;
      }
    }

    inputs.connected = camera.isConnected();
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }
}
