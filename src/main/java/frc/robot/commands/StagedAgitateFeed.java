package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRunMode;

public class StagedAgitateFeed extends Command {

    Intake intake;

    public Rotation2d position;
    public double wheelSpeed;

    public boolean done;

    private static double maxPositionDeg = 100;

    public static Timer timer = new Timer();
    int count = 0;

    public StagedAgitateFeed(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        position = intake.getMode().getSetpoint();
        count = 0;
        timer.restart();
        done = false;
    }

    @Override
    public void execute() {

        if (!done) {
            if (count % 2 == 0 && timer.advanceIfElapsed(0.4)) {
                position = position.plus(Rotation2d.fromDegrees(20));
                count++;
            } else if (count % 2 == 1 && timer.advanceIfElapsed(0.4)) {
                position = position.minus(Rotation2d.fromDegrees(10));
                count++;
            }
        } else {
            // duplicate for future changes
            if (count % 2 == 0 && timer.advanceIfElapsed(0.4)) {
                position = position.plus(Rotation2d.fromDegrees(20));
                count++;
            } else if (count % 2 == 1 && timer.advanceIfElapsed(0.4)) {
                position = position.minus(Rotation2d.fromDegrees(20));
                count++;
            }
        }

        position = Rotation2d.fromDegrees(MathUtil.clamp(position.getDegrees(), 0, maxPositionDeg));
        if (position.getDegrees() == maxPositionDeg) {
            done = true;
        }

        intake.setCustomMode(position, IntakeRunMode.AGITATE_2.intakeDutyCycle);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeRunMode.UP);
    }
}
