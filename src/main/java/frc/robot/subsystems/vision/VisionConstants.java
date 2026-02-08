package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // --- Vision Config ---

  // Set cameraName on PhotonVision web interface. Edit camera name from camera
  // type to camera
  // position. To find robotToCamera, measure the distance from the camera to the
  // center of the
  // robot or use the robot's CAD model.

  // Docs:
  // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html

  enum CameraPosition {
    FRONT,
    UNKNOWN,
    TOP,
    BACK,
    LEFT,
    RIGHT;
  }

  public record CameraConfig(
      String cameraName, CameraPosition cameraPosition, Transform3d robotToCamera) {}

  public static final CameraConfig SIM_FRONT_CAMERA =
      new CameraConfig(
          "front_camera",
          CameraPosition.FRONT,
          new Transform3d(
              new Translation3d(Units.inchesToMeters(27.5 / 2.0 + 1.0), 0, Units.inchesToMeters(6)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));

  public static final CameraConfig TOP_CAMERA =
      new CameraConfig(
          "top_camera",
          CameraPosition.TOP,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-6), Units.inchesToMeters(12.5), Units.inchesToMeters(21)),
              new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(0))));

  public static final CameraConfig RIGHT_CAMERA =
      new CameraConfig(
          "right_camera",
          CameraPosition.RIGHT,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-8), Units.inchesToMeters(-12.5), Units.inchesToMeters(8)),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-75))));
  public static final CameraConfig BACK_CAMERA =
      new CameraConfig(
          "back_camera",
          CameraPosition.BACK,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-12.5),
                  Units.inchesToMeters(-12.5),
                  Units.inchesToMeters(8)),
              new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-150))));
  public static final CameraConfig LEFT_CAMERA =
      new CameraConfig(
          "left_camera",
          CameraPosition.LEFT,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-12.5), Units.inchesToMeters(-8), Units.inchesToMeters(8)),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-225))));
}
