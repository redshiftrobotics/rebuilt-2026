package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // --- Vision Config ---

  // Set cameraName on PhotonVision web interface. Edit camera name from camera
  // type to camera position. To find robotToCamera, measure the distance from
  // the camera to the center of the robot or use the robot's CAD model.

  // Docs:
  // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html

  enum CameraPositionName {
    FRONT,
    LAUNCHER_LEFT,
    UNKNOWN,
  }

  public record CameraConfig(
      String cameraName, CameraPositionName cameraPosition, Transform3d robotToCamera) {}

  public static final CameraConfig TOP_CAMERA_ANGLED =
      new CameraConfig(
          "spencercam",
          CameraPositionName.LAUNCHER_LEFT,
          fromOnShape(10.470732, 2.344, 16.864 + 3.710392, -20, 0));

  public static Transform3d fromOnShape(
      double xInches, double yInches, double zInches, double pitchDegrees, double yawDegrees) {
    // x and y are flipped because of the different coordinate system conventions between OnShape
    // and PhotonVision. Y must be negated, corresponding to a 90 degree rotation
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-yInches),
            Units.inchesToMeters(xInches),
            Units.inchesToMeters(zInches)),
        new Rotation3d(
            0, Units.degreesToRadians(pitchDegrees), Units.degreesToRadians(yawDegrees)));
  }
}
