package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class DropSlapdown extends Command {

    private final Intake intake;

    public DropSlapdown(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.slapdownSet(0.1);
    }

    
}