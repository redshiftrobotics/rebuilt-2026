package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SlapdownIO {
  @AutoLog
  public class SlapdownIOInputsAutoLogged {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  public default void updateInputs(SlapdownIOInputsAutoLogged inputs) {}

  public abstract void setMotorIdleMode(IdleMode idleMode);

  public default void setPID(double kp, double ki, double kd) {}

  public default void setSpeed(double speed) {}

  public default void setSetpoint(Rotation2d setPoint) {}

  public default void stopMotor() {}
}
