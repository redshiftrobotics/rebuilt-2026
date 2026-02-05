package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeWheelIOSim implements IntakeWheelIO {
  private final DCMotorSim sim;
  private final DCMotor motor;

  public IntakeWheelIOSim() {
    motor = DCMotor.getKrakenX60(1);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.004, IntakeConstants.WHEEL_GEAR_RATIO),
            motor);
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
  public double getSpeed() {
    return sim.getAngularVelocityRPM();
  }

  @Override
  public void stop() {
    sim.setAngularVelocity(0);
  }
}
