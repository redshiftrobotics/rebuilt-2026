package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
  public void stop() {
    this.speed = RotationsPerSecond.of(0);
  }

  @Override
  public boolean isAtSetpoint() {
    return true;
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    this.speed = speed;
  }
}
