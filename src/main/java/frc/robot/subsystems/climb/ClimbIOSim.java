package frc.robot.subsystems.climb;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Simulation implementation of the ClimbIO. */
public class ClimbIOSim implements ClimbIO {
  /** Position threshold for "limit switch" (radians). */
  private static final double IS_DOWN_THRESHOLD_RADIANS = 0.1;

  private final DCMotorSim sim;

  // Visualization:
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d climbArm;

  public ClimbIOSim() {
    // Set up sim

    DCMotor motor = DCMotor.getNEO(1);
    LinearSystem<N2, N1, N2> dcMotorSystem =
        LinearSystemId.createDCMotorSystem(motor, 0.004, ClimbConstants.GEAR_RATIO);

    sim = new DCMotorSim(dcMotorSystem, motor);

    // Set up vis

    mechanism = new LoggedMechanism2d(2, 3);
    root = mechanism.getRoot("ClimbRoot", 1, 0.5);
    climbArm = new LoggedMechanismLigament2d("ClimbArm", 2.0, 90, 10, new Color8Bit(Color.kBlue));
    root.append(climbArm);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Periodic
    sim.update(0.02);
    climbArm.setAngle(Units.radiansToDegrees(inputs.positionRad));

    // Log inputs
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = new double[] {sim.getInputVoltage()};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.climberDown = isAtBottom();

    // Log visualization
    Logger.recordOutput("Climb/Visualization", mechanism);
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
