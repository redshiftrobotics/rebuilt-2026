package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeWheelConstants;

public class IntakeCommands {
  public static Command extendSlapdown(Intake intake) {
    return intake.runOnce(() -> intake.setSlapdownSetpoint(intake.getSavedDownSetpoint()));
  }

  public static Command retractSlapdown(Intake intake) {
    return intake.runOnce(() -> intake.setSlapdownSetpoint(intake.getSavedUpSetpoint()));
  }

  public static Command startIntake(Intake intake) {
    return intake.runOnce(() -> intake.setWheelSpeed(IntakeWheelConstants.SPEED_INTAKING));
  }

  public static Command stopIntake(Intake intake) {
    return intake.runOnce(intake::stopWheels);
  }

  private IntakeCommands() {
    // utility class doesn't need to be instantiated.
  }

  // Increments
  public static Command incrementUpSlapdown(Intake intake, Rotation2d amount) {
    return Commands.runOnce(
        () -> intake.setSavedUpSetpoint(intake.getSavedUpSetpoint().plus(amount)));
  }

  public static Command incrementDownSlapdown(Intake intake, Rotation2d amount) {
    return intake.runOnce(
        () -> intake.setSavedDownSetpoint(intake.getSavedDownSetpoint().plus(amount)));
  }
}
