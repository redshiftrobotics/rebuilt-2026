package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class DropSlapdownNoPid extends Command {

    private final Intake intake;

    public DropSlapdownNoPid(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.slapdownSet(0.5);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        intake.slapdownStop();
    }
}