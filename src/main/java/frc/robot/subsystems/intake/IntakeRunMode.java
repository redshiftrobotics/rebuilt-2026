package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeRunMode {
    START_POSITION(Rotation2d.fromDegrees(105), 0),
    INTAKING(Rotation2d.fromDegrees(10), 1.0),
    OUTTAKING(Rotation2d.fromDegrees(20), -1.0),
    UP(Rotation2d.fromDegrees(90), 0.0);

    private final Rotation2d setpoint;
    private Rotation2d shift;

    public final double intakeDutyCycle;

    private IntakeRunMode(Rotation2d setpoint, double intakeDutyCycle) {
        this.setpoint = setpoint;
        this.intakeDutyCycle = intakeDutyCycle;
        resetShift();
    }

    public void resetShift() {
        this.shift = Rotation2d.kZero;
    }

    public void shiftSetpoint(Rotation2d shift) {
        this.shift = this.shift.plus(shift);
    }

    public Rotation2d getSetpoint() {
        return setpoint.plus(shift);
    }

    @Override
    public String toString() {
        return String.format(
                "%s(wrist=%.2f deg, intake=%.2f)", name(), getSetpoint().getDegrees(), intakeDutyCycle);
    }
}
