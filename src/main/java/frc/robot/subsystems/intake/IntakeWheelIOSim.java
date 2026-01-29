package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeWheelIOSim implements IntakeWheelIO {
  private DCMotor motor;
  private DCMotorSim sim;

  public IntakeWheelIOSim() {
    motor = DCMotor.getNEO(1);
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, 1.0), motor);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = new double[] {0.0};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    sim.setAngularVelocity(velocityRadPerSec);
  }

  @Override
  public void stop() {
    sim.setAngularVelocity(0);
  }
}