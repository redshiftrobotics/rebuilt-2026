package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.SparkUtil;

public class SlapdownIOSparkMax implements SlapdownIO {

  private Rotation2d slapdownUpPosition;
  private Rotation2d slapdownDownPosition;

  private final SparkMax motor;
  private final SparkClosedLoopController motorPID;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;

  private final Debouncer connectionDebouncer = new Debouncer(0.5);

  public SlapdownIOSparkMax(SparkMax motor) {
    slapdownUpPosition = IntakeConstants.SLAPDOWN_UP_SETPOINT;
    slapdownDownPosition = IntakeConstants.SLAPDOWN_DOWN_SETPOINT;

    this.motor = motor;

    absoluteEncoder = motor.getAbsoluteEncoder();

    relativeEncoder = motor.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition());

    motorPID = motor.getClosedLoopController();

    SparkBaseConfig config =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(IntakeConstants.SLAPDOWN_WHEEL_INVERTED)
            .voltageCompensation(12);

    config
        .encoder
        .positionConversionFactor(IntakeConstants.SLAPDOWN_GEAR_RATIO)
        .velocityConversionFactor(IntakeConstants.SLAPDOWN_GEAR_RATIO);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
    SparkUtil.clearError();

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
        values -> inputs.appliedVolts = new double[] {values});
    SparkUtil.ifOk(
        motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = new double[] {value});

    inputs.motorConnected = connectionDebouncer.calculate(!motor.hasStickyFault());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setSetpoint(Rotation2d setPoint) {
    motorPID.setSetpoint(setPoint.getRotations(), ControlType.kVelocity);
  }

  @Override
  public void setSavedUpSetpoint(Rotation2d setPoint) {
    motorPID.setSetpoint(setPoint.getRotations(), ControlType.kVelocity);

    slapdownUpPosition = setPoint;
  }

  @Override
  public void setSavedDownSetpoint(Rotation2d setPoint) {
    motorPID.setSetpoint(setPoint.getRotations(), ControlType.kVelocity);

    slapdownDownPosition = setPoint;
  }

  @Override
  public Rotation2d getSavedUpSetpoint() {
    return slapdownUpPosition;
  }

  @Override
  public Rotation2d getSavedDownSetpoint() {
    return slapdownDownPosition;
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
