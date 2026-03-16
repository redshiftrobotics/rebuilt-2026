package frc.robot.subsystems.led;

public class LEDStripIOSim implements LEDStripIO {

  private BlinkinLEDPattern pattern;
  private boolean runSetup;

  public LEDStripIOSim(BlinkinLEDPattern initialPattern) {
    pattern = initialPattern;
  }

  @Override
  public void updateInputs(LEDStripIOInputs inputs) {
    inputs.runningSetup = runSetup;
    inputs.requestedPattern = pattern;

    inputs.targetPulse = inputs.requestedPattern.getPulse();
    if (runSetup) {
      inputs.targetPulse = -1;
    }

    inputs.measuredPulse = inputs.targetPulse;
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
