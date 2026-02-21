package frc.robot.subsystems.common.velocityMotor;

import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for motor hardware */
public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    boolean motorConnected = true;

    double positionRad = 0.0;
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double supplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(MotorIOInputs inputs);

  /** Run the motor at the specified amount. */
  public void setOpenLoop(double output);

  /** Run to velocity setpoint */
  public default void setVelocity(double velocityRadsPerSec) {
    setVelocity(velocityRadsPerSec, 0);
  }

  /** Run to velocity setpoint with feedforward */
  public void setVelocity(double velocityRadsPerSec, double feedforward);

  /** Configure PID */
  public void setPID(double kP, double kI, double kD);

  /** Enable or disable brake mode on the motor. */
  public void setBrakeMode(boolean enable);

  /** Disable output to brake and turn motor */
  public void stop();
}
