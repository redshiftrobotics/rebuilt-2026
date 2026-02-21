package frc.robot.subsystems.music;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonOrchestra extends SubsystemBase {
  private static final String SONG_PATH_1 = "pirate.chrp";
  private static final String SONG_PATH_2 = "clash.chrp";

  private final Orchestra orchestra;
  private int numberInstruments = 0;

  private final String songPath;

  private static final TalonOrchestra instance = new TalonOrchestra();

  public static TalonOrchestra getInstance() {
    return instance;
  }

  private TalonOrchestra() {
    this.songPath = SONG_PATH_1;
    orchestra = new Orchestra(songPath);
  }

  public void addInstrument(CommonDevice instrument) {
    orchestra.addInstrument(instrument);
    numberInstruments++;
  }

  public Command playSong() {
    return Commands.startEnd(orchestra::play, orchestra::stop)
        .ignoringDisable(true)
        .alongWith(
            Commands.print(
                String.format("Playing %s with %d instruments", songPath, numberInstruments)));
  }
}
