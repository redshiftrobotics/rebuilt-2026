package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.SparkUtil;

public class IntakeWheelIOSparkMax implements IntakeWheelIO {
  private final SparkMax motor;
  private final RelativeEncoder relativeEncoder;
  private final Debouncer connectionDebouncer = new Debouncer(0.5);

  public IntakeWheelIOSparkMax(SparkMax motor) {
    this.motor = motor;

    relativeEncoder = motor.getEncoder();

    SparkBaseConfig config =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(IntakeConstants.INTAKE_WHEEL_INVERTED)
            .voltageCompensation(12);

    config
        .encoder
        .positionConversionFactor(IntakeConstants.INTAKE_WHEEL_GEAR_RATIO)
        .velocityConversionFactor(IntakeConstants.SLAPDOWN_GEAR_RATIO);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    SparkUtil.clearStickyFault();

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
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public double getSpeed() {
    return motor.get();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
