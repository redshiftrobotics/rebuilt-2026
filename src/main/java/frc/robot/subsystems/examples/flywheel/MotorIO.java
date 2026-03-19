package frc.robot.subsystems.examples.flywheel;

import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for motor hardware */
public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public boolean motorConnected = true;
    public boolean pushedConfigFault = false;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedDutycycle = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorIOInputs inputs) {}

  /** Run the motor at duty cycle. */
  public default void setDutyCycle(double dutyCycle) {}

  /** Run the motor at the specified amount. Either voltage or torque current. */
  public default void setOpenLoop(double output) {}

  /** Run to velocity setpoint */
  public default void setVelocity(double velocityRadsPerSec) {
    setVelocity(velocityRadsPerSec, 0);
  }

  /** Run to velocity setpoint with feedforward */
  public default void setVelocity(double velocityRadsPerSec, double arbFeedforward) {}

  /** Configure PID */
  public default void setPID(PIDConfig pidConfig) {}

  /** Configure FF */
  public default void setFF(FeedForwardConfigRecord ffConfig) {}

  /** Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {}

  /** Disable output to brake and turn motor */
  public default void stop() {}
}
