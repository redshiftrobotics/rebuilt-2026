package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class DropSlapdown extends Command {

  private final Intake intake;

  public DropSlapdown(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.slapdownSetPoint(IntakeConstants.SLAPDOWN_DOWN_SETPOINT);
  }

  @Override
  public boolean isFinished() {
    return intake.slapdownIsAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    intake.slapdownStop();
  }
}
