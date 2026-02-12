package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommands {
  public static Command extendSlapdown(Intake intake) {
    return Commands.runOnce(
        () -> intake.setSlapdownSetpoint(IntakeConstants.SLAPDOWN_DOWN_SETPOINT), intake);
  }

  public static Command retractSlapdown(Intake intake) {
    return Commands.runOnce(
        () -> intake.setSlapdownSetpoint(IntakeConstants.SLAPDOWN_UP_SETPOINT), intake);
  }

  public static Command startIntake(Intake intake) {
    return Commands.runOnce(() -> intake.setWheelSpeed(IntakeConstants.INTAKE_WHEEL_SPEED), intake);
  }

  public static Command stopIntake(Intake intake) {
    return Commands.runOnce(() -> intake.stopWheels(), intake);
  }

  private IntakeCommands() {
    // utility class doesn't need to be instantiated.
  }
}
