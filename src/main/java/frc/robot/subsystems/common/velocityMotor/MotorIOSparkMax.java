package frc.robot.subsystems.common.velocityMotor;

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

/** Motor IO implementation for SparkMax motor controller */
public class MotorIOSparkMax implements MotorIO {

  private final SparkMax motor;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController feedback;

  private boolean brakeMode = true;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public MotorIOSparkMax(VelocityMotorConstants constants) {

    motor = new SparkMax(constants.deviceId(), MotorType.kBrushless);
    relativeEncoder = motor.getEncoder();
    feedback = motor.getClosedLoopController();

    brakeMode = constants.brakeMode();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit((int) constants.stallCurrent())
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(constants.gearRatio())
        .velocityConversionFactor(constants.gearRatio());
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> relativeEncoder.setPosition(0.0));
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
    inputs.motorConnected = connectedDebouncer.calculate(!SparkUtil.hasError());
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setVoltage(output);
  }

  @Override
  public void setVelocity(double velocityRadsPerSec, double feedforward) {
    feedback.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadsPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (brakeMode != enable) {
      brakeMode = enable;
      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
      tryUntilOk(
          motor,
          5,
          () ->
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
