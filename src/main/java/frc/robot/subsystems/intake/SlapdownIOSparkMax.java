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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.SparkUtil;
import frc.robot.utility.records.PIDConfig;

public class SlapdownIOSparkMax implements SlapdownIO {

  private final SparkMax motor;
  private final SparkClosedLoopController motorPID;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;

  private final Debouncer connectionDebouncer = new Debouncer(0.5);

  public SlapdownIOSparkMax(SparkMax motor) {

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
    inputs.encodersAligned =
        MathUtil.isNear(relativeEncoder.getVelocity(), absoluteEncoder.getPosition(), 0.1);
  }

  @Override
  public void setPID(PIDConfig pid) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(pid.kP(), pid.kI(), pid.kD());
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setSetpoint(Rotation2d setPoint) {
    motorPID.setSetpoint(setPoint.getRotations(), ControlType.kVelocity);
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
