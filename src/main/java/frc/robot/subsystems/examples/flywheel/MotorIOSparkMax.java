package frc.robot.subsystems.examples.flywheel;

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
import frc.robot.utility.SparkUtil;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

/** Motor IO implementation for SparkMax motor controller */
public class MotorIOSparkMax implements MotorIO {

  private final SparkMax motor;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController feedback;

  private final SparkMaxConfig config = new SparkMaxConfig();

  private boolean brakeMode = true;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public MotorIOSparkMax(MotorConstants constants) {

    motor = new SparkMax(constants.deviceId(), MotorType.kBrushless);
    relativeEncoder = motor.getEncoder();
    feedback = motor.getClosedLoopController();

    brakeMode = constants.brakeMode();

    config
        .idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit((int) constants.stallCurrent())
        .voltageCompensation(12.0)
        .inverted(constants.inverted());
    config
        .encoder
        .positionConversionFactor(constants.gearRatio())
        .velocityConversionFactor(constants.gearRatio());
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);

    tryUntilOk(motor, 5, () -> relativeEncoder.setPosition(0.0));

    pushConfig();
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {

    SparkUtil.clearError();
    ifOk(
        motor,
        relativeEncoder::getPosition,
        value -> inputs.positionRad = Units.rotationsToRadians(value));
    ifOk(
        motor,
        relativeEncoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = value);
    ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);
    ifOk(motor, motor::getAppliedOutput, value -> inputs.appliedDutycycle = value);
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
    // config.closedLoop.feedForward.sva(ffConfig.kS(), ffConfig.kV(), ffConfig.kA());
    // pushConfig();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (brakeMode != enable) {
      brakeMode = enable;
      config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
      pushConfig();
    }
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  private void pushConfig() {
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
