package frc.robot.subsystems.hopper.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {}

  /** Updates the set of loggable inputs */
  public default void updateInputs(FeederIOInputs inputs) {}
}
