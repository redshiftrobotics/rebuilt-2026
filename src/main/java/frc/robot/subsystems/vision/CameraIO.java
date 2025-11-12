package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionConstants.CameraPosition;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    int numPipelineResults;
    int numEstimates;
    boolean connected = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(CameraIOInputs inputs) {}

  default List<EstimatedRobotPose> getEstimates() {
    return null;
  }

  /** Get name of io camera */
  default String getCameraName() {
    return "";
  }

  /** Get position of camera on robot */
  default CameraPosition getCameraPosition() {
    return CameraPosition.UNKNOWN;
  }

  default void setLastRobotPose(Pose2d pose) {}

  /** Set april tag field layout to use */
  default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {}
}
