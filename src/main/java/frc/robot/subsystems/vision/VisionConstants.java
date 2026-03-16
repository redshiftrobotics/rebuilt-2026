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
    UNKNOWN,
    TOP,
    TOP_FLAT,
    TOP_ANGLE,
    BACK,
    LEFT,
    RIGHT
  }

  public record CameraConfig(
      String cameraName, CameraPositionName cameraPosition, Transform3d robotToCamera) {}

  // --- 2026 REBUILT ---

  public static final CameraConfig TOP_CAMERA_ANGLED =
      new CameraConfig(
          "spencercam",
          CameraPositionName.TOP_ANGLE,
          fromOnShape(-10.470732, 2.076919, 16.956726 + 3.710392, -20, 0));

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

  // --- Old ---

  public static final CameraConfig SIM_FRONT_CAMERA =
      new CameraConfig(
          "front_camera",
          CameraPositionName.FRONT,
          new Transform3d(
              new Translation3d(Units.inchesToMeters(27.5 / 2.0 + 1.0), 0, Units.inchesToMeters(6)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));

  public static final CameraConfig TOP_CAMERA =
      new CameraConfig(
          "top_camera",
          CameraPositionName.TOP,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-6), Units.inchesToMeters(12.5), Units.inchesToMeters(21)),
              new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(0))));

  public static final CameraConfig RIGHT_CAMERA =
      new CameraConfig(
          "right_camera",
          CameraPositionName.RIGHT,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-8), Units.inchesToMeters(-12.5), Units.inchesToMeters(8)),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-75))));
  public static final CameraConfig BACK_CAMERA =
      new CameraConfig(
          "back_camera",
          CameraPositionName.BACK,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-12.5),
                  Units.inchesToMeters(-12.5),
                  Units.inchesToMeters(8)),
              new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-150))));
  public static final CameraConfig LEFT_CAMERA =
      new CameraConfig(
          "left_camera",
          CameraPositionName.LEFT,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-12.5), Units.inchesToMeters(-8), Units.inchesToMeters(8)),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-225))));
}
