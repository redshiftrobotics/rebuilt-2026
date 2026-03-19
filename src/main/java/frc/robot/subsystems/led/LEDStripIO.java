package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for april tag detection systems */
public interface LEDStripIO {
  @AutoLog
  public static class LEDStripIOInputs {
    int targetPulse;
    int measuredPulse;
    boolean runningSetup = false;

    BlinkinLEDPattern requestedPattern;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(LEDStripIOInputs inputs) {}

  public default void setPattern(BlinkinLEDPattern pattern) {}

  public default void runSetup(boolean run) {}
}
