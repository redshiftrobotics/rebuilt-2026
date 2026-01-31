package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SlapdownIO {
  @AutoLog
  public class SlapdownIOInputsAutoLogged {
    public double PositionRad = 0.0;
    public double VelocityRadPerSec = 0.0;

    public double[] AppliedVolts = new double[] {};
    public double[] SupplyCurrentAmps = new double[] {};
  }

  public default void updateInputs(SlapdownIOInputsAutoLogged inputs) {}

  public default void setMotorMode() {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setSpeed(double speed) {}

  public default void setSetpoint(Rotation2d setPoint) {}

  public default void stopMotor() {}
}
