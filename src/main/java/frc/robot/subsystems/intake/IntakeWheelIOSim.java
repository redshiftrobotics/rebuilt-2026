package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeWheelConstants;

/** Simulation implementation of IntakeWheelIO. */
public class IntakeWheelIOSim implements IntakeWheelIO {
  private final DCMotorSim sim;
  private final DCMotor motor;

  public IntakeWheelIOSim() {
    motor = DCMotor.getNEO(1);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.00004, IntakeWheelConstants.GEAR_RATIO),
            motor);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.motorConnected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setSpeed(double speed) {
    sim.setInputVoltage(speed * 12);
  }

  @Override
  public double getSpeed() {
    return sim.getAngularVelocityRPM();
  }

  @Override
  public void stop() {
    sim.setInput(0);
    sim.setAngularVelocity(0);
  }
}
