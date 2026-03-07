package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Launcher subsystem's hood. */
public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public boolean isAdjustable = false;
    public double positionLeft = 0.0;
    public double positionRight = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double position) {}
}
