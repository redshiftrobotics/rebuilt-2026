package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.HopperConstants.MotorConstants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

/** Physics sim implementation of motor IO. */
public class HopperMotorIOSim implements HopperMotorIO {

    private final DCMotorSim sim;

    private double appliedVolts = 0.0;

    private final PIDController feedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);
    private final SimpleMotorFeedforward feedfoward =
            new SimpleMotorFeedforward(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);

    private boolean closedLoop = false;
    private double ffVolts = 0;

    public HopperMotorIOSim(DCMotorSim sim) {
        this.sim = sim;
    }

    public HopperMotorIOSim(DCMotor motor, MotorConstants config, double JKgMetersSquared) {
        this(new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, 1.0 / config.gearRatio()), motor));
    }

    @Override
    public void updateInputs(HopperMotorIOInputs inputs) {

        if (closedLoop) {
            appliedVolts = feedback.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts;
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
        inputs.appliedDutycycle = appliedVolts / 12.0;
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
    public void setVelocity(double velocityRadsPerSec, double arbFeedforward) {
        closedLoop = true;
        ffVolts = feedfoward.calculate(velocityRadsPerSec) + arbFeedforward;
        feedback.setSetpoint(velocityRadsPerSec);
    }

    @Override
    public void setPID(PIDConfig pidConfig) {
        feedback.setPID(pidConfig.kP(), pidConfig.kI(), pidConfig.kD());
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
