package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

/** Simulation implementation of the TemplateIO. */
public class IntakeIOSim implements IntakeIO {

  private double speed = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec = speed * Units.rotationsPerMinuteToRadiansPerSecond(5676);
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
