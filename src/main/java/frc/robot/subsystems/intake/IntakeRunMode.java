package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeRunMode {
    START_POSITION(Rotation2d.fromDegrees(105), 0),
    INTAKING(Rotation2d.fromDegrees(0), 1.0),
    INTAKING_NO_WHEELS(Rotation2d.fromRadians(0), 0),
    OUTTAKING_DUMP(Rotation2d.fromDegrees(3), -1.0),
    POST_INTAKE_TRANSITION(Rotation2d.fromDegrees(88), 1.0),
    AGITATE_1_UP(Rotation2d.fromDegrees(88), 1.0),
    AGITATE_2(Rotation2d.fromDegrees(60), 0.3),
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
