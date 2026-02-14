package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class IntakeWheelIOSparkMax implements IntakeWheelIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public IntakeWheelIOSparkMax(SparkMax motor) {
    this.motor = motor;
    
    encoder = motor.getEncoder();

    SparkBaseConfig config =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(IntakeConstants.INTAKE_WHEEL_INVERTED)
            .voltageCompensation(12);

    config.encoder
        .positionConversionFactor(IntakeConstants.INTAKE_WHEEL_GEAR_RATIO)
        .velocityConversionFactor(IntakeConstants.SLAPDOWN_GEAR_RATIO);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
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
