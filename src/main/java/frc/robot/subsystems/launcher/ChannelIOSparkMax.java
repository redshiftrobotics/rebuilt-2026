package frc.robot.subsystems.launcher;

import static frc.robot.utility.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.launcher.LauncherConstants.ChannelConstants.ChannelConfig;
import frc.robot.utility.SparkUtil;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

/** Channel IO implementation for SparkMax motor controller */
public class ChannelIOSparkMax implements ChannelIO {

    private final SparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final SparkClosedLoopController feedback;

    private final SparkMaxConfig config = new SparkMaxConfig();

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public ChannelIOSparkMax(String name, ChannelConfig constants) {

        motor = new SparkMax(constants.deviceId(), MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        feedback = motor.getClosedLoopController();

        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .voltageCompensation(12.0)
                .inverted(constants.inverted());
        config.encoder.positionConversionFactor(constants.gearRatio()).velocityConversionFactor(constants.gearRatio());
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);

        tryUntilOk(motor, 5, () -> relativeEncoder.setPosition(0.0));

        pushConfig();
    }

    @Override
    public void updateInputs(ChannelIOInputs inputs) {

        SparkUtil.clearError();
        ifOk(
                motor,
                relativeEncoder::getVelocity,
                value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
        ifOk(motor, () -> motor.getAppliedOutput() * motor.getBusVoltage(), value -> inputs.appliedVolts = value);
        ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);
        ifOk(motor, motor::getAppliedOutput, value -> inputs.appliedDutyCycle = value);
        inputs.motorConnected = connectedDebouncer.calculate(!SparkUtil.hasError());
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }

    @Override
    public void setOpenLoop(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadsPerSec, double arbFeedforward) {
        feedback.setSetpoint(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadsPerSec),
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                arbFeedforward,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setPID(PIDConfig pidConfig) {
        config.closedLoop.pid(pidConfig.kP(), pidConfig.kI(), pidConfig.kD());
        pushConfig();
    }

    @Override
    public void setFF(FeedForwardConfigRecord ffConfig) {
        config.closedLoop.feedForward.sva(ffConfig.kS(), ffConfig.kV(), ffConfig.kA());
        pushConfig();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private void pushConfig() {
        tryUntilOk(
                motor,
                5,
                () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}
