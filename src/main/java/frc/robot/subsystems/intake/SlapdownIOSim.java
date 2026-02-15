package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SlapdownIOSim implements SlapdownIO {

  private Rotation2d slapdownUpPosition;
  private Rotation2d slapdownDownPosition;

  private final DCMotorSim sim;
  private final DCMotor motor;

  private final PIDController pidController;

  public SlapdownIOSim() {
    slapdownUpPosition = IntakeConstants.SLAPDOWN_UP_SETPOINT;
    slapdownDownPosition = IntakeConstants.SLAPDOWN_DOWN_SETPOINT;

    motor = DCMotor.getNEO(1);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.004, IntakeConstants.SLAPDOWN_GEAR_RATIO),
            motor);

    pidController =
        new PIDController(
            IntakeConstants.SLAPDOWN_PID.kP(),
            IntakeConstants.SLAPDOWN_PID.kI(),
            IntakeConstants.SLAPDOWN_PID.kD());

    pidController.setTolerance(0);
  }

  @Override
  public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    if (!pidController.atSetpoint()) {
      sim.setInput(pidController.calculate(sim.getAngularPositionRad()));
    }

    inputs.motorConnected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = new double[] {sim.getInputVoltage()};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }

  @Override
  public void setSetpoint(Rotation2d setPoint) {
    pidController.setSetpoint(setPoint.getRadians());
  }

  @Override
  public void setSavedUpSetpoint(Rotation2d setPoint) {
    pidController.setSetpoint(setPoint.getRadians());

    slapdownUpPosition = setPoint;
  }

  @Override
  public void setSavedDownSetpoint(Rotation2d setPoint) {
    pidController.setSetpoint(setPoint.getRadians());

    slapdownDownPosition = setPoint;
  }

  @Override
  public Rotation2d getSavedUpSetpoint() {
    return slapdownUpPosition;
  }

  @Override
  public Rotation2d getSavedDownSetpoint() {
    return slapdownDownPosition;
  }

  @Override
  public void stopMotor() {
    sim.setInput(0);
    sim.setAngularVelocity(0);
  }
}
