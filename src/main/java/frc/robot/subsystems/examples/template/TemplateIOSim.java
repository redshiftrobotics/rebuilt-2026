package frc.robot.subsystems.examples.template;

import edu.wpi.first.math.util.Units;

/** Simulation implementation of the TemplateIO. */
public class TemplateIOSim implements TemplateIO {

  private double speed = 0.0;

  public TemplateIOSim() {}

  @Override
  public void updateInputs(TemplateIOInputs inputs) {
    inputs.velocityRadPerSec = speed * Units.rotationsPerMinuteToRadiansPerSecond(5676);
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
