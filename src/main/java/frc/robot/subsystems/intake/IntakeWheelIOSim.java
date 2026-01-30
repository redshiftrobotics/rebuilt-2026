package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeWheelIOSim implements IntakeWheelIO {
  private DCMotor gearbox;
  private DCMotorSim sim;

  public IntakeWheelIOSim() {
    gearbox = DCMotor.getNEO(1);
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, IntakeConstants.JKgMetersSquared, IntakeConstants.gearing), gearbox);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = new double[] {0.0};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void set(double speed) {
    sim.setInput(speed);
  }

  @Override
  public void stop() {
    sim.setAngularVelocity(0);
  }
}
