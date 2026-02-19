package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConfig;

public class HopperMotorIOSim implements HopperMotorIO {
  /* Motor and sim */
  private final DCMotor motor = DCMotor.getNEO(1);
  private final DCMotorSim sim;

  /* PID controller */
  private final PIDController pidController;

  /* Voltage state storage */
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  public HopperMotorIOSim(double gearRatio) {
    // Create motor
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, gearRatio), motor);

    // Create PID controller
    pidController = new PIDController(0, 0, 0);
  }

  @Override
  public void setPID(PIDConfig pid) {
    // Apply values to PID controller
    pidController.setPID(pid.kP(), pid.kI(), pid.kD());
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // Set PID controller setpoint
    pidController.setSetpoint(velocityRadPerSec);

    // Store feedforward voltage
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    // Stop by setting velocity to 0
    setVelocity(0.0, 0.0);
  }

  @Override
  public void updateInputs(HopperMotorIOInputs inputs) {
    // Set new input voltage
    appliedVolts =
        MathUtil.clamp(
            pidController.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);

    // Update simulation state
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    // Fetch new values
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }
}
