package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

/** Simulation implementation of the TemplateIO. */
public class ChannelIOSim implements ChannelIO {

  private AngularVelocity speed = AngularVelocity.ofBaseUnits(0, RadiansPerSecond);

  public ChannelIOSim() {}

  @Override
  public void updateInputs(ChannelIOInputs inputs) {
    inputs.velocityRadPerSec = speed.in(RadiansPerSecond);
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    this.speed = speed;
  }
}
