package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SlapdownIOSim implements SlapdownIO {

  private final DCMotorSim sim;
  private final DCMotor motor;

  public SlapdownIOSim() {
    motor = DCMotor.getKrakenX60(1);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.004, IntakeConstants.SLAPDOWN_GEAR_RATIO),
            motor);
  }

  @Override
  public void updateInputs(SlapdownIOInputs inputs) {
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = new double[] {0.0};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setPID(double kP, double kI, double kD) {}

  @Override
  public void setSetpoint(Rotation2d setPoint) {
    sim.setAngle(setPoint.getRadians());
  }

  @Override
  public void stopMotor() {
    sim.setAngularVelocity(0);
  }
}
