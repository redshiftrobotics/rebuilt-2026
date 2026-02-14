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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SlapdownIOSparkMax implements SlapdownIO {

  private final SparkMax motor;
  private final SparkClosedLoopController motorPID;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;

  public SlapdownIOSparkMax(SparkMax motor, AbsoluteEncoder absoluteEncoder) {
    this.motor = motor;
    this.absoluteEncoder = absoluteEncoder;

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
    inputs.positionRad = Units.rotationsToRadians(absoluteEncoder.getPosition());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());

    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
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
  public void stopMotor() {
    motor.stopMotor();
  }
}
