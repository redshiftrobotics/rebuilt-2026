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
          "neilcam",
          CameraPositionName.TOP_ANGLE,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(0.257),
                  Units.inchesToMeters(10.015),
                  Units.inchesToMeters(19.137)),
              new Rotation3d(0, Units.degreesToRadians(-90 + 70), 0)));

  public static final CameraConfig TOP_CAMERA_FLAT =
      new CameraConfig(
          "a2",
          CameraPositionName.TOP_FLAT,
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(0.158),
                  Units.inchesToMeters(-10.015),
                  Units.inchesToMeters(18.096)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));
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
  public static final CameraConfig METAL_BOT_2_FRONT =
      new CameraConfig(
          "neilcam",
          CameraPositionName.FRONT,
          new Transform3d(
              new Translation3d(Units.inchesToMeters(28.5 / 2.0), 0, Units.inchesToMeters(7.6)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));
  public static final CameraConfig METAL_BOT_2_BACK =
      new CameraConfig(
          "geraldcam",
          CameraPositionName.BACK,
          new Transform3d(
              new Translation3d(-Units.inchesToMeters(28.5 / 2.0), 0, Units.inchesToMeters(7.6)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));
}
