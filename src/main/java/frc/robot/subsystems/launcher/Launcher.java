package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final ChannelIO[] channelIOs;
  private final ChannelIOInputsAutoLogged[] channelInputs;

  private Supplier<Translation2d> hubPosition = null;
  private Supplier<Translation2d> robotVelocity = null;

  private boolean running = false;

  /** Creates a new Template. */
  public Launcher(ChannelIO... channelIOs) {
    hoodIO =
        switch (LauncherConstants.HOOD_TYPE) {
          case FIXED -> new HoodIOFixed() {};
          case ACTUATOR -> new HoodIOActuator();
        };

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
      Supplier<Translation2d> hubPositionSupplier, Supplier<Translation2d> robotVelocitySupplier) {
    hubPosition = hubPositionSupplier;
    robotVelocity = robotVelocitySupplier;
  }

  @Override
  public void periodic() {
    if (running) {
      ShotParameters parameters = ShotCalculator.method1(hubPosition.get(), robotVelocity.get());

      hoodIO.setLaunchAngle(parameters.pitch());
      for (ChannelIO channel : channelIOs) {
        channel.setSpeed(
            RadiansPerSecond.of(
                parameters.velocity().in(MetersPerSecond)
                    / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters)
                    * LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER));
      }
    } else {
      for (ChannelIO channel : channelIOs) {
        channel.stop();
      }
    }

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Launcher/Hood", hoodInputs);

    for (int i = 0; i < channelIOs.length; i++) {
      channelIOs[i].updateInputs(channelInputs[i]);
      Logger.processInputs("Launcher/Channel" + String.valueOf(i), channelInputs[i]);
    }
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
