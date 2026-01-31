package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class SlapdownIOSparkMax implements SlapdownIO {

  private final SparkMax motor;
  private final SparkClosedLoopController motorPID;
  private final RelativeEncoder encoder;

  public SlapdownIOSparkMax(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    motorPID = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
    inputs.PositionRad =
        Units.rotationsPerMinuteToRadiansPerSecond(
            encoder.getVelocity() / IntakeConstants.SLAPDOWN_GEAR_RATIO);
    inputs.VelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            encoder.getVelocity() / IntakeConstants.SLAPDOWN_GEAR_RATIO);

    inputs.AppliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.SupplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setMotorMode() {}

  @Override
  public void setPID(double kp, double ki, double kd) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kp, ki, kd);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void setSetpoint(double setPointRad) {
    motorPID.setSetpoint(setPointRad, ControlType.kVelocity); // check this
    // TODO idk
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
