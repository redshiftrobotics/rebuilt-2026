package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {
  private final HoodActuatorIO hoodIO;
  private final HoodActuatorIOInputsAutoLogged hoodInputs = new HoodActuatorIOInputsAutoLogged();
  private final ChannelIO[] channelIOs;
  private final ChannelIOInputsAutoLogged[] channelInputs;

  private Translation2d hubPosition;
  private Translation2d robotVelocity;

  /** Creates a new Template. */
  public Launcher(HoodActuatorIO io, ChannelIO... channelIOs) {
    this.hoodIO = io;

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

  @Override
  public void periodic() {

    ShotParameters parameters = ShotCalculator.method1(hubPosition, robotVelocity);

    for (ChannelIO channel : channelIOs) {
      // Give flywheel additional velocity to account for velocity lost in momentum transfer
      // Give flywheel double  velocity to account for spin. It rolls the ball, rather than pushing

      channel.setSpeed(
          RadiansPerSecond.of(
              parameters.velocity().in(MetersPerSecond)
                  / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters)
                  * LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER));
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
    return false;
    // TODO
  }
}
