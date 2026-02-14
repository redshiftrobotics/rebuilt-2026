package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Climb subsystem. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};

    public boolean climberDown = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run open loop at the specified percentage power. */
  public default void setSpeed(double speed) {}

  /**
   * Check if the climber is all the way retracted.
   * @return True if the motor is allthe way down.
   */
	public default boolean isAtBottom() { return false; }

  /** Stop the motor and lock the climber in place. */
  public default void stop() {}
}
