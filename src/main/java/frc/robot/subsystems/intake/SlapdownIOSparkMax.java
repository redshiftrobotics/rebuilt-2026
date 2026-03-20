package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.intake.IntakeConstants.SlapdownConstants;
import frc.robot.utility.SparkUtil;
import frc.robot.utility.records.PIDConfig;

/** SparkMAX implementation of SlapdownIO. */
public class SlapdownIOSparkMax implements SlapdownIO {

  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;

  private final Debouncer connectionDebouncer = new Debouncer(0.5);
  private final Debouncer encoderIsGood = new Debouncer(0.5, DebounceType.kBoth);

  public SlapdownIOSparkMax() {

    this.motor = new SparkMax(SlapdownConstants.CAN_ID, MotorType.kBrushless);

    relativeEncoder = motor.getEncoder();
    absoluteEncoder = motor.getAbsoluteEncoder();

    pid = motor.getClosedLoopController();

    SparkBaseConfig config =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(SlapdownConstants.MOTOR_INVERTED)
            .smartCurrentLimit(30)
            .voltageCompensation(12);

    config
        .encoder
        .positionConversionFactor(SlapdownConstants.GEAR_RATIO)
        .velocityConversionFactor(SlapdownConstants.GEAR_RATIO);

    config
        .absoluteEncoder
        .inverted(SlapdownConstants.ABSOLUTE_ENCODER_INVERTED)
        .zeroOffset(SlapdownConstants.ABSOLUTE_ENCODER_ZERO.getRotations());

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    relativeEncoder.setPosition(absoluteEncoder.getPosition());
  }

  @Override
  public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
    SparkUtil.clearError();

    if (DriverStation.isDisabled()) {
      relativeEncoder.setPosition(absoluteEncoder.getPosition());
    }

    SparkUtil.ifOk(
        motor,
        relativeEncoder::getPosition,
        value -> inputs.positionRad = Units.rotationsToRadians(value));
    SparkUtil.ifOk(
        motor,
        relativeEncoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    SparkUtil.ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = value);
    SparkUtil.ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);

    SparkUtil.ifOk(
        motor,
        absoluteEncoder::getPosition,
        value -> inputs.absolutePositionRad = Units.rotationsToRadians(value));
    SparkUtil.ifOk(
        motor,
        absoluteEncoder::getVelocity,
        value ->
            inputs.absoluteVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));

    inputs.motorConnected = connectionDebouncer.calculate(!SparkUtil.hasError());
    inputs.encodersAligned =
        encoderIsGood.calculate(
            MathUtil.isNear(relativeEncoder.getPosition(), absoluteEncoder.getPosition(), 0.1));
  }

  @Override
  public void setPID(PIDConfig pid) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(pid.kP(), pid.kI(), pid.kD(), ClosedLoopSlot.kSlot0);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setSetpoint(Rotation2d setpoint) {
    pid.setSetpoint(setpoint.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPIDSlotSecondary(PIDConfig pid) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(pid.kP(), pid.kI(), pid.kD(), ClosedLoopSlot.kSlot1);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setSetpointSecondary(Rotation2d setpoint) {
    pid.setSetpoint(setpoint.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
