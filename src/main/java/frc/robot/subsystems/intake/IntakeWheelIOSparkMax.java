package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;


public class IntakeWheelIOSparkMax implements IntakeWheelIO {
  double GEAR_RATIO = 1;
  private final SparkMax motor = new SparkMax(0, MotorType.kBrushless);
  private final SparkBaseConfig config;
  private final RelativeEncoder encoder;

  public IntakeWheelIOSparkMax() {
    encoder = motor.getEncoder();

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setVelocity(double velocity) {
    motor.set(velocity);
  }

  @Override
  public void stop() {
    motor.set(0);
  }
}