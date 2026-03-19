package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionConstants.CameraPositionName;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhotonVision implements CameraIO {
    protected final PhotonCamera camera;

    private final PhotonPoseEstimator photonPoseEstimator;
    private final CameraPositionName cameraPosition;
    private final Transform3d robotToCamera;

    private final List<EstimatedRobotPose> estimates = new ArrayList<>();

    public CameraIOPhotonVision(CameraConfig config) {
        this.cameraPosition = config.cameraPosition();
        this.robotToCamera = config.robotToCamera();

        // --- Setup Camera ---
        camera = new PhotonCamera(config.cameraName());

        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOff);

        // --- Setup Pose Estimator ---

        // Turn on 3d mode for camera for this to work
        // This is only possible if camera is calibrated for the current specified resolution
        // https://drive.google.com/file/d/1I_Jj1JweqhHpK0TEI80Y15qQ3NN_TmBx/view?usp=sharing

        // MULTI_TAG_PNP_ON_COPROCESSOR:
        // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#enabling-multitag

        // PhotonPoseEstimator:
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#using-a-photonposeestimator

        photonPoseEstimator = new PhotonPoseEstimator(
                FieldConstants.emptyFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.robotToCamera());

        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public CameraPositionName getCameraPosition() {
        return cameraPosition;
    }

    @Override
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    @Override
    public void setAprilTagFieldLayout(AprilTagFieldLayout fieldTags) {
        photonPoseEstimator.setFieldTags(fieldTags);
    }

    public void setLastRobotPose(Pose2d lastRobotPose) {
        photonPoseEstimator.setLastPose(lastRobotPose);
    }

    @Override
    public List<EstimatedRobotPose> getEstimates() {
        return estimates;
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {

        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

        inputs.numPipelineResults = pipelineResults.size();

        estimates.clear();

        for (PhotonPipelineResult result : pipelineResults) {
            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);
            estimatedRobotPose.ifPresent(estimates::add);
        }

        inputs.numEstimates = estimates.size();

        inputs.connected = camera.isConnected();
    }

    @Override
    public String getCameraName() {
        return camera.getName();
    }
}
