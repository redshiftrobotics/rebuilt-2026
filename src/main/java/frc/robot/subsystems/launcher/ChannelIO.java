package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utility.records.FeedForwardConfigRecord;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface ChannelIO {
  @AutoLog
  public static class ChannelIOInputs {
    public boolean motorConnected = true;
    public boolean pushedConfigFault = false;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedDutycycle = 0.0;
  }

  /** Updates the set of loggable inputs. */
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ChannelIOInputs inputs) {}

  /** Run the motor at duty cycle. */
  public default void setDutyCycle(double dutyCycle) {}

  /** Run the motor at the specified amount. Either voltage or torque current. */
  public default void setOpenLoop(double output) {}

  /** Run to velocity setpoint */
  public default void setVelocity(AngularVelocity velocity) {
    setVelocity(velocity, 0);
  }

  /** Run to velocity setpoint with feedforward */
  public default void setVelocity(AngularVelocity velocity, double arbFeedforward) {}

  /** Configure PID */
  public default void setPID(double kP, double kI, double kD) {}

  /** Configure FF */
  public default void setFF(FeedForwardConfigRecord ffConfig) {}

  /** Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {}

  /** Disable output to brake and turn motor */
  public default void stop() {}

  public default boolean isAtSetpoint() {
    return false;
  }
}
