package frc.robot.subsystems.led;

public class LEDConstants {

  public static final BlinkinLEDPattern DEFAULT_PATTERN = BlinkinLEDPattern.OFF;

  public enum BlinkenMode {
    STRIP_5V(2125),
    STRIP_12V(2145);

    public final int setupPulse;

    BlinkenMode(int setupPulse) {
      this.setupPulse = setupPulse;
    }
  }

  public record LEDConfig(int pwmChannel, BlinkenMode mode) {}

  public static final LEDConfig LEDS_STRIP_2025_LEFT = new LEDConfig(0, BlinkenMode.STRIP_5V);
  public static final LEDConfig LEDS_STRIP_2025_RIGHT = new LEDConfig(1, BlinkenMode.STRIP_5V);
}
