package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRunMode;

public class StagedAgitateFeed extends Command {

    private static final double maxPositionDeg = 110;

    private final Intake intake;
    public final Timer timer = new Timer();

    private Rotation2d position;

    private boolean done;
    private int count = 0;

    public StagedAgitateFeed(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        position = intake.getMode().getSetpoint();
        count = 0;
        timer.restart();
        done = false;

        IntakeRunMode.AGITATE_CUSTOM.resetShift();
        intake.setMode(IntakeRunMode.AGITATE_CUSTOM);
    }

    @Override
    public void execute() {

        if (!done) {
            if (count % 2 == 0 && timer.advanceIfElapsed(0.4)) {
                position = position.plus(Rotation2d.fromDegrees(25));
                count++;
            } else if (count % 2 == 1 && timer.advanceIfElapsed(0.4)) {
                position = position.minus(Rotation2d.fromDegrees(-10));
                count++;
            }
        } else {
            // duplicate for future changes
            if (count % 2 == 0 && timer.advanceIfElapsed(0.3)) {
                position = IntakeRunMode.AGITATE_1_UP.getSetpoint();
                count++;
            } else if (count % 2 == 1 && timer.advanceIfElapsed(0.5)) {
                position = IntakeRunMode.AGITATE_2.getSetpoint();
                count++;
            }
        }

        position = Rotation2d.fromDegrees(MathUtil.clamp(position.getDegrees(), 0, maxPositionDeg));
        if (position.getDegrees() == maxPositionDeg) {
            done = true;
        }

        // Same as setting
        IntakeRunMode.AGITATE_CUSTOM.shiftSetpoint(position.minus(IntakeRunMode.AGITATE_CUSTOM.getSetpoint()));

        intake.setMode(IntakeRunMode.AGITATE_CUSTOM);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeRunMode.UP);
    }
}
