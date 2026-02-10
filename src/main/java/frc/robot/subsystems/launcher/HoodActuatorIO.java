package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface HoodActuatorIO {
  @AutoLog
  public static class HoodActuatorIOInputs {
    public double position;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodActuatorIOInputs inputs) {}
}
