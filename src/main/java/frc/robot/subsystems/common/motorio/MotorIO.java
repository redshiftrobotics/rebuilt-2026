package frc.robot.subsystems.common.motorio;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utility.records.PIDConfig;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setTargetPosition(Rotation2d targetPosition) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set PID configuration. */
  public default void configurePID(double Kp, double Ki, double Kd) {}

  /** Set PID configuration. */
  public default void configurePID(PIDConfig config) {}
}
