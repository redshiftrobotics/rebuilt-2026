package frc.robot.subsystems.launcher;

import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Launcher subsystem's channel. */
public interface ChannelIO {
  @AutoLog
  public static class ChannelIOInputs {
    public boolean motorConnected = true;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedDutyCycle = 0.0;
  }

  public default String getName() {
    return "";
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ChannelIOInputs inputs) {}

  /** Run the motor at duty cycle. */
  public default void setDutyCycle(double dutyCycle) {}

  /** Run the motor at the specified amount. Either voltage or torque current. */
  public default void setOpenLoop(double output) {}

  /** Run to velocity setpoint */
  public default void setVelocity(double radPerSec) {
    setVelocity(radPerSec, 0);
  }

  /** Run to velocity setpoint with feedforward */
  public default void setVelocity(double radPerSec, double arbFeedforward) {}

  /** Configure PID */
  public default void setPID(PIDConfig pid) {}

  /** Configure FF */
  public default void setFF(FeedForwardConfigRecord ffConfig) {}

  /** Disable output to brake and turn motor */
  public default void stop() {}
}
