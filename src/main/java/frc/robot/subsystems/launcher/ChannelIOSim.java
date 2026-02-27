package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.subsystems.launcher.LauncherConstants.ChannelConstants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

/** Physics sim implementation of motor IO. */
public class ChannelIOSim implements ChannelIO {

  private final String name;

  private final FlywheelSim sim;

  private double appliedVolts = 0.0;

  private final PIDController feedback =
      new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);
  private final SimpleMotorFeedforward feedfoward =
      new SimpleMotorFeedforward(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);

  private boolean closedLoop = false;
  private double FFVolts = 0;

  public ChannelIOSim(String name, ChannelConstants constants) {
    this.name = name;
    final DCMotor motor = DCMotor.getKrakenX60(1);
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                motor, LauncherConstants.FLYWHEEL_MOI.baseUnitMagnitude(), constants.gearRatio()),
            motor);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(ChannelIOInputs inputs) {

    if (closedLoop) {
      appliedVolts = feedback.calculate(sim.getAngularVelocityRadPerSec()) + FFVolts;
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
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.appliedDutyCycle = appliedVolts / 12.0;
  }

  @Override
  public void setDutyCycle(double output) {
    setOpenLoop(output * 12);
  }

  @Override
  public void setOpenLoop(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(AngularVelocity velocity, double arbFeedforward) {
    closedLoop = true;
    FFVolts = feedfoward.calculate(velocity.in(RadiansPerSecond)) + arbFeedforward;
    feedback.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setPID(PIDConfig pid) {
    feedback.setPID(pid.kP(), pid.kI(), pid.kD());
  }

  @Override
  public void setFF(FeedForwardConfigRecord ffConfig) {
    feedfoward.setKs(ffConfig.kS());
    feedfoward.setKv(ffConfig.kV());
    feedfoward.setKa(ffConfig.kA());
  }

  @Override
  public void setBrakeMode(boolean enable) {}

  @Override
  public void stop() {
    setOpenLoop(0);
  }
}
