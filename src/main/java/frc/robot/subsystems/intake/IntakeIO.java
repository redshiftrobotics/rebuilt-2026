package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified percentage power. */
  public default void setSpeed(double speed) {}
}
