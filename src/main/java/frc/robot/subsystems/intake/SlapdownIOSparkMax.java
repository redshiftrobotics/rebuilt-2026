package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
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

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IntakeConstants.SLAPDOWN_BRAKE_MODE);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
    inputs.positionRad =
        Units.rotationsToRadians(
            absoluteEncoder.getPosition() / IntakeConstants.SLAPDOWN_GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            absoluteEncoder.getVelocity() / IntakeConstants.SLAPDOWN_GEAR_RATIO);

    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setMotorIdleMode(IdleMode idleMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(idleMode);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void setSetpoint(Rotation2d setPoint) {
    motorPID.setSetpoint(setPoint.getRotations(), ControlType.kVelocity);
  }

  @Override
  public boolean slapdownIsAtSetpoint() {
    return motorPID.isAtSetpoint();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
