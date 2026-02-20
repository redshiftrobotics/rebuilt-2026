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

  private final DCMotorSim driveSim;

  private double driveAppliedVolts = 0.0;

  private final PIDController driveFeedback;

  private boolean driveClosedLoop = false;
  private double driveFFVolts = 0;

  private MotorIOSim(DCMotorSim driveMotor) {
    this.driveSim = driveMotor;

    this.driveFeedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);
  }

  public MotorIOSim(DCMotor motor, VelocityMotorConfig config, double JKgMetersSquared) {
    this(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, config.gearRatio()),
            motor));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {

    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec()) + driveFFVolts;
    } else {
      driveFeedback.reset();
    }

    if (DriverStation.isDisabled()) {
      driveAppliedVolts = 0.0;
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    driveSim.update(Constants.LOOP_PERIOD_SECONDS);

    // --- Drive ---
    inputs.driveMotorConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setDriveVelocity(double velocityRadsPerSec, double feedforward) {
    driveClosedLoop = true;
    driveFFVolts = feedforward;
    driveFeedback.setSetpoint(velocityRadsPerSec);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void stop() {
    setDriveOpenLoop(0);
  }
}
