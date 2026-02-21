package frc.robot.subsystems.common.velocityMotor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Physics sim implementation of motor IO. */
public class MotorIOSim implements MotorIO {

  private final DCMotorSim sim;

  private double appliedVolts = 0.0;

  private final PIDController feedback;

  private boolean closedLoop = false;
  private double FFVolts = 0;

  private MotorIOSim(DCMotorSim motor) {
    this.sim = motor;
    this.feedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);
  }

  public MotorIOSim(DCMotor motor, VelocityMotorConstants config, double JKgMetersSquared) {
    this(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, config.gearRatio()),
            motor));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {

    if (closedLoop) {
      appliedVolts =
          feedback.calculate(sim.getAngularVelocityRadPerSec()) + FFVolts;
    } else {
      feedback.reset();
    }

    if (DriverStation.isDisabled()) {
      appliedVolts = 0.0;
    }

    // Update simulation state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    // --- Drive ---
    inputs.motorConnected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double velocityRadsPerSec, double feedforward) {
    closedLoop = true;
    FFVolts = feedforward;
    feedback.setSetpoint(velocityRadsPerSec);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    feedback.setPID(kP, kI, kD);
  }

  @Override
  public void setBrakeMode(boolean enable) {}

  @Override
  public void stop() {
    setOpenLoop(0);
  }
}
