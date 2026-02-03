package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SetIntake extends Command {
  private final Intake intake;
  private final double speed;

  public SetIntake(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    intake.wheelSet(speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
