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
    UNKNOWN,
    LAUNCHER_RIGHT,
    BACK_SWERVE_RIGHT,
    BACK_SWERVE_LEFT,
  }

  public record CameraConfig(
      String cameraName, CameraPositionName cameraPosition, Transform3d robotToCamera) {}

  // --- 2026 REBUILT ---

  // To find camera position, measure from Arducam model's "Lens Location" mate connector to the
  // center x/y (forward/left) of robot and z (height) being from floor.
  // If origin is not center of robot on the floor, then add some offset so it is.

  // NOTE: Measure tool might not always display correct signs, check by logging cameraPose of
  // camera on 3d field with robot and confirming things look right

  // CAD Link:
  // https://cad.onshape.com/documents/21a71e9819567f3a143f688d/w/9023f69f68be4b352eda45dd/e/00d63fdcbb0aeb3dcf2a86a6

  // Example screenshot:
  // https://drive.google.com/file/d/1uEEOQu9T5Yfil7LBZX7JSF_0oiMpAjLz/view?usp=sharing

  public static final CameraConfig LAUNCHER_RIGHT_CAMERA =
      new CameraConfig(
          "spencercam",
          CameraPositionName.LAUNCHER_RIGHT,
          fromOnShape(-10.470732, 2.344, 16.864 + 3.710392, -20, 0));

  public static final CameraConfig BACK_SWERVE_LEFT_CAMERA =
      new CameraConfig(
          "TODO1", // geraldcam?
          CameraPositionName.BACK_SWERVE_LEFT,
          fromOnShape(+10.047429, 10.817071, 4.096002 + 3.710392, -25, 180 - 45));

  public static final CameraConfig BACK_SWERVE_RIGHT_CAMERA =
      new CameraConfig(
          "TODO2", // neilcam?
          CameraPositionName.BACK_SWERVE_RIGHT,
          fromOnShape(-10.047429, 10.817071, 4.096002 + 3.710392, -25, 180 + 45));

  /**
   * Convert from OnShape coordinates to PhotonVision coordinates.
   * 
   * @param xInches inches from center of robot in Onshape's x direction (left- / right+)
   * @param yInches inches from center of robot in Onshape's y direction (forward- / backward+)
   * @param zInches inches from floor in Onshape's z direction (up+ /down-)
   * @param pitchDegrees rotation around robot's left-right axis. 0 when level looking level to horizon, negative pitch looking up, positive pitch looking down
   * @param yawDegrees rotation around robot's front-back axis. 0 when camera is facing forward, positive yaw looking left, negative yaw looking right
   * @return Transform3d in WPILib's coordinate system
   */
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
