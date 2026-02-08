package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * This command never 'isFinished', because it is canceled by the command
 * scheduler when the trigger is lifted. (The command is executed using
 * whileTrue)
 */
public class IntakeCommand extends Command {

  private final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setSlapdownSetpoint(IntakeConstants.SLAPDOWN_DOWN_SETPOINT);
    intake.setWheelSpeed(IntakeConstants.INTAKE_WHEEL_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSlapdownSetpoint(IntakeConstants.SLAPDOWN_UP_SETPOINT);
    intake.stopWheels();
  }
}
