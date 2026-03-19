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

  public static final LEDConfig LED_STRIP_BACK = new LEDConfig(2, BlinkenMode.STRIP_5V);
  public static final LEDConfig LED_STRIP_1X1 = new LEDConfig(3, BlinkenMode.STRIP_5V);
  public static final LEDConfig LED_STRIP_FUTURE = new LEDConfig(4, BlinkenMode.STRIP_5V);
}
