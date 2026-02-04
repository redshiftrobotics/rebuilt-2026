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
  private final SparkBaseConfig config;
  private final RelativeEncoder encoder;

  public IntakeWheelIOSparkMax(SparkMax motor) {
    this.motor = motor;
    encoder = motor.getEncoder();

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition() / IntakeConstants.WHEEL_GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            encoder.getVelocity() / IntakeConstants.WHEEL_GEAR_RATIO);
    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void set(double speed) {
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
