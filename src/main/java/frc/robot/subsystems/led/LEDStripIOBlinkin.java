package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.subsystems.led.LEDConstants.BlinkenMode;
import frc.robot.subsystems.led.LEDConstants.LEDConfig;

/**
 * Implementation of LEDStripIO for REV Blinkin LED Driver. Uses PWM to set the pattern.
 * 
 * {@link https://www.revrobotics.com/rev-11-1105/} 
 */
public class LEDStripIOBlinkin implements LEDStripIO {

  private final PWM pwm;
  private final BlinkenMode mode;

  private BlinkinLEDPattern pattern;
  private boolean runSetup;

  public LEDStripIOBlinkin(LEDConfig config, BlinkinLEDPattern initialPattern) {
    pwm = new PWM(config.pwmChannel());
    mode = config.mode();
    pattern = initialPattern;
  }

  @Override
  public void updateInputs(LEDStripIOInputs inputs) {
    inputs.runningSetup = runSetup;
    inputs.requestedPattern = pattern;

    inputs.targetPulse = inputs.requestedPattern.getPulse();
    if (runSetup) {
      inputs.targetPulse = mode.setupPulse;
    }

    pwm.setPulseTimeMicroseconds(inputs.targetPulse);

    inputs.measuredPulse = pwm.getPulseTimeMicroseconds();
  }

  @Override
  public void runSetup(boolean run) {
    runSetup = run;
  }

  @Override
  public void setPattern(BlinkinLEDPattern pattern) {
    this.pattern = pattern;
  }
}
