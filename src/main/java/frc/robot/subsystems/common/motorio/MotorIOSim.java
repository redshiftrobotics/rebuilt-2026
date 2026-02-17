package frc.robot.subsystems.common.motorio;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utility.records.PIDConfig;

public class MotorIOSim implements MotorIO {
  private final DCMotor motor = DCMotor.getNEO(1);
  private final LinearSystem<N2, N1, N2> simSysId;
  private final DCMotorSim sim;
  private final PIDController pid;

  public MotorIOSim(double gearing) {
    simSysId = LinearSystemId.createDCMotorSystem(motor, 0.004, gearing);
    sim = new DCMotorSim(simSysId, motor);
    pid = new PIDController(0.1, 0.0, 0.0);
  }

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngularPositionRotations()), -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setTargetPosition(Rotation2d targetPosition) {
    closedLoop = true;
    pid.setSetpoint(targetPosition.getRotations());
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double Kp, double Ki, double Kd) {
    pid.setPID(Kp, Ki, Kd);
  }

  @Override
  public void configurePID(PIDConfig config) {
    pid.setPID(config.kP(), config.kI(), config.kD());
  }
}
