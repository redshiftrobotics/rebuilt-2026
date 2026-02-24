package frc.robot.subsystems.climb;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of the ClimbIO. */
public class ClimbIOSim implements ClimbIO {
  /** Position threshold for "limit switch" (radians). */
  private static final double IS_DOWN_THRESHOLD_RADIANS = 0.1;

  private final DCMotorSim sim;

  public ClimbIOSim() {
    // Set up sim

    DCMotor motor = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> dcMotorSystem =
        LinearSystemId.createDCMotorSystem(motor, 0.004, ClimbConstants.GEAR_RATIO);

    sim = new DCMotorSim(dcMotorSystem, motor);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Periodic
    sim.update(0.02);

    // Log inputs
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = new double[] {sim.getInputVoltage()};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.climberDown = isAtBottom();
  }

  @Override
  public void setSpeed(double speed) {
    sim.setInputVoltage(speed * 12.0); // robot is 12v
  }

  @Override
  public void stop() {
    sim.setInputVoltage(0.0);
  }

  @Override
  public boolean isAtBottom() {
    return sim.getAngularPositionRad() <= IS_DOWN_THRESHOLD_RADIANS;
  }
}
