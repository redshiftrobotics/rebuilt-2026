package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.CameraPosition;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    EstimatedRobotPose[] poseEstimates;
    boolean connected = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(CameraIOInputs inputs) {}

  /** Get name of io camera */
  default String getCameraName() {
    return "Camera";
  }

  default CameraPosition getCameraPosition() {
    return CameraPosition.UNKNOWN;
  }

  /** Set april tag field layout to use */
  default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {}
}
