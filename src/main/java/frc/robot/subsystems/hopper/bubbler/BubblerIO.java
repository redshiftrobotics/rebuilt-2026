package frc.robot.subsystems.hopper.bubbler;

import org.littletonrobotics.junction.AutoLog;

public interface BubblerIO {
  @AutoLog
  public static class BubblerIOInputs {}

  /** Updates the set of loggable inputs */
  public default void updateInputs(BubblerIOInputs inputs) {}
}
