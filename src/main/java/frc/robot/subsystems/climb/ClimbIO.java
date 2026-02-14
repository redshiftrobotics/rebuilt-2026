package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Climb subsystem. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double velocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run open loop at the specified percentage power. */
  public default void setSpeed(double speed) {}
}
