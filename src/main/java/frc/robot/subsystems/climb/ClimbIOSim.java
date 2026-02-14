package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;

/** Simulation implementation of the ClimbIO. */
public class ClimbIOSim implements ClimbIO {

  private double speed = 0.0;

  public ClimbIOSim() {}

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.velocityRadPerSec = speed * Units.rotationsPerMinuteToRadiansPerSecond(5676);
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }
}
