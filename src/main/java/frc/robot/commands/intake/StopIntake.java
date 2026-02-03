package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class StopIntake extends Command {
  public final Intake intake;

  public StopIntake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.wheelStop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
