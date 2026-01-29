package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.bubbler.BubblerIO;
import frc.robot.subsystems.hopper.bubbler.BubblerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.feeder.FeederIO;
import frc.robot.subsystems.hopper.feeder.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* IO layers */
  private final BubblerIO bubbler;
  private final FeederIO feeder;

  /* Loggable inputs */
  private final BubblerIOInputsAutoLogged bubblerInputs = new BubblerIOInputsAutoLogged();
  private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  public Hopper(BubblerIO bubblerIO, FeederIO feederIO) {
    bubbler = bubblerIO;
    feeder = feederIO;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    bubbler.updateInputs(bubblerInputs);
    feeder.updateInputs(feederInputs);
    Logger.processInputs("Hopper/Bubbler", bubblerInputs);
    Logger.processInputs("Hopper/Feeder", feederInputs);
  }
}
