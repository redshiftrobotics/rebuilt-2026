package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final ChannelIO[] channelIOs;
  private final ChannelIOInputsAutoLogged[] channelInputs;

  private Supplier<Pose2d> robotPose = null;
  private Supplier<Translation2d> robotVelocity = null;

  private boolean running = false;
  private Optional<Rotation2d> robotYaw = Optional.empty();

  /** Creates a new Template. */
  public Launcher(HoodIO hoodIO, ChannelIO ...channelIOs) {
    this.hoodIO = hoodIO;

    this.channelIOs = channelIOs;
    channelInputs = new ChannelIOInputsAutoLogged[channelIOs.length];
    for (int i = 0; i < channelIOs.length; i++) {
      this.channelIOs[i].configurePID(
          LauncherConstants.FLYWHEEL_KP,
          LauncherConstants.FLYWHEEL_KI,
          LauncherConstants.FLYWHEEL_KD);
      channelInputs[i] = new ChannelIOInputsAutoLogged();
    }
  }

  public void stop() {
    this.running = false;
  }

  public void start() {
    this.running = true;
  }

  public void setRunning(boolean enabled) {
    this.running = enabled;
  }

  public void configure(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> robotVelocitySupplier) {
    robotPose = robotPoseSupplier;
    robotVelocity = robotVelocitySupplier;
  }

  @Override
  public void periodic() {
    if (running) {
      Translation2d hubTranslation = FieldConstants.Hub.topCenterPoint.toTranslation2d().minus(robotPose.get().getTranslation());
      ShotParameters parameters =
          ShotCalculator.method1(hubTranslation, robotVelocity.get(), hoodIO.hoodType());

      hoodIO.setAngle(parameters.pitch());
      for (ChannelIO channel : channelIOs) {
        channel.setSpeed(
            RadiansPerSecond.of(
                parameters.velocity().in(MetersPerSecond)
                    / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters)
                    * LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER));
      }
      robotYaw = Optional.of(parameters.yaw());
    } else {
      for (ChannelIO channel : channelIOs) {
        channel.stop();
      }
      robotYaw = Optional.empty();
    }

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Launcher/Hood", hoodInputs);

    for (int i = 0; i < channelIOs.length; i++) {
      channelIOs[i].updateInputs(channelInputs[i]);
      Logger.processInputs("Launcher/Channel" + String.valueOf(i), channelInputs[i]);
    }
  }

  public Optional<Rotation2d> getRobotYaw() {
    return robotYaw;
  }

  // Function to determine if the launcher wheels and hood are at their setpoints
  public boolean isReady() {
    boolean ready = true;
    for (ChannelIO channelIO : channelIOs) {
      ready = ready && channelIO.isAtSetpoint();
    }
    return ready;
  }
}
