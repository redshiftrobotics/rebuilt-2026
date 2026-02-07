package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommand extends Command {

  private final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.slapdownSetPoint(IntakeConstants.SLAPDOWN_DOWN_SETPOINT);
    intake.wheelSet(IntakeConstants.INTAKE_WHEEL_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.slapdownSetPoint(IntakeConstants.SLAPDOWN_UP_SETPOINT);
    intake.wheelStop();
  }
}
