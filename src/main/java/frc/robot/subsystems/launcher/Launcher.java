package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {
  private final HoodActuatorIO hoodIO;
  private final HoodActuatorIOInputsAutoLogged hoodInputs = new HoodActuatorIOInputsAutoLogged();
  private final ChannelIO[] channelIOs;
  private final ChannelIOInputsAutoLogged[] channelInputs;

  /** Creates a new Template. */
  public Launcher(HoodActuatorIO io, ChannelIO... channelIOs) {
    this.hoodIO = io;

    this.channelIOs = channelIOs;
    channelInputs = new ChannelIOInputsAutoLogged[channelIOs.length];
    for (int i = 0; i < channelIOs.length; i++) {
      this.channelIOs[i].configurePID(LauncherConstants.FLYWHEEL_KP, LauncherConstants.FLYWHEEL_KI, LauncherConstants.FLYWHEEL_KD);
      channelInputs[i] = new ChannelIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
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
