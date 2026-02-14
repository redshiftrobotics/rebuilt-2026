package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
  @AutoLog
  public class SlapdownIOInputs {
    public boolean motorConnected = false;

    public double positionRad = IntakeConstants.SLAPDOWN_UP_SETPOINT.getRadians();
    public double velocityRadPerSec = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  public default void updateInputs(SlapdownIOInputsAutoLogged inputs) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setSetpoint(Rotation2d setPoint) {}

  public default void stopMotor() {}
}
