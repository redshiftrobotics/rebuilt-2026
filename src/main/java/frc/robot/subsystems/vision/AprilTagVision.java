package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Camera.ProcessedEstimatedRobotPose;
import frc.robot.utility.Elastic;
import frc.robot.utility.Elastic.Notification;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final Camera[] cameras;

  private Consumer<ProcessedEstimatedRobotPose> visionPoseConsumer;

  private boolean hasSuccessfulEstimation = false;

  public AprilTagVision(Supplier<Pose2d> robotPoseSupplier, CameraIO... camerasIO) {
    this.cameras =
        Arrays.stream(camerasIO)
            .map(io -> new Camera(io, robotPoseSupplier))
            .toArray(Camera[]::new);
  }

  /** Set a consumer to receive all vision poses as they are processed */
  public void setVisionPoseConsumer(Consumer<ProcessedEstimatedRobotPose> visionPoseConsumer) {
    this.visionPoseConsumer = visionPoseConsumer;
  }

  /** Set the AprilTag field layout for all cameras */
  public void setAprilTagFieldLayout(AprilTagFieldLayout layout) {
    for (Camera camera : cameras) {
      camera.setAprilTagFieldLayout(layout);
    }
  }

  /** Enable pose filtering for all cameras. Only enable if you are providing a good robot pose */
  public void enablePoseFiltering(boolean filterBasedOnLastPose, boolean filterBasedOnGyro) {
    for (Camera camera : cameras) {
      camera.enablePoseFiltering(filterBasedOnLastPose, filterBasedOnGyro);
    }
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

    // Loop through all cameras
    for (Camera camera : cameras) {

      Logger.recordOutput("Vision/" + camera.getCameraPosition() + "/name", camera.getCameraName());

      // Loop through all results that the camera has
      for (ProcessedEstimatedRobotPose result : camera.getResults()) {

        (result.status().isSuccess() ? robotPosesAccepted : robotPosesRejected)
            .add(result.estimatedPose());

        seenTagPoses.addAll(result.tagPositionsOnField());

        Logger.recordOutput("Vision/" + camera.getCameraPosition() + "/status", result.status());
        Logger.recordOutput(
            "Vision/" + camera.getCameraPosition() + "/estimatedPose", result.estimatedPose());
        Logger.recordOutput(
            "Vision/" + camera.getCameraPosition() + "/standardDeviations",
            result.standardDeviations().getData());
        Logger.recordOutput(
            "Vision/" + camera.getCameraPosition() + "/timestampSeconds",
            result.timestampSeconds());
        Logger.recordOutput(
            "Vision/" + camera.getCameraPosition() + "/tagPositionsOnField",
            result.tagPositionsOnField().toArray(Pose3d[]::new));

        if (visionPoseConsumer != null) {
          visionPoseConsumer.accept(result);
        }
      }
    }

    hasSuccessfulEstimation = !robotPosesAccepted.isEmpty();

    Logger.recordOutput("Vision/robotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/robotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/seenTagPoses", seenTagPoses.toArray(Pose3d[]::new));
  }

  /** Get whether or not the vision system has a valid estimate */
  public boolean hasSuccessfulEstimate() {
    return hasSuccessfulEstimation;
  }

  /** Send a command to restart the PhotonVision program on the given IP address */
  public static void restartPhotonVision(String ipString) {
    sendPhotonVisionCommand(ipString, "restartProgram");
  }

  /** Send a command to reboot the PhotonVision device on the given IP address */
  public static void rebootPhotonVision(String ipString) {
    sendPhotonVisionCommand(ipString, "restartDevice");
  }

  private static void sendPhotonVisionCommand(String ipString, String command) {
    try {
      HttpClient httpClient = HttpClient.newHttpClient();
      HttpRequest request =
          HttpRequest.newBuilder()
              .uri(new URI("http://" + ipString + ":5800/api/utils/" + command))
              .POST(HttpRequest.BodyPublishers.noBody())
              .build();
      httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString());
    } catch (Exception exception) {
      Elastic.sendNotification(
          new Notification(
              Notification.NotificationLevel.INFO,
              "PhotonVision Command Error",
              String.format(
                  "Failed to send command %s to PhotonVision at %s due to %s",
                  command, ipString, exception.getMessage())));
    }
  }
}
