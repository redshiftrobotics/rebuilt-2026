package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeConstants.SlapdownConstants;
import frc.robot.utility.SparkUtil;
import frc.robot.utility.records.PIDConfig;

public class HopperMotorIOSparkMax implements HopperMotorIO {

  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private final RelativeEncoder encoder;

  private final Debouncer connectionDebouncer = new Debouncer(0.5);

  public HopperMotorIOSparkMax(int motorID, double gearRatio) {
    this.motor = new SparkMax(SlapdownConstants.CAN_ID, MotorType.kBrushless);

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    SparkBaseConfig config =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(SlapdownConstants.INVERTED)
            .smartCurrentLimit(37)
            .voltageCompensation(12);

    config.encoder.positionConversionFactor(gearRatio).velocityConversionFactor(gearRatio);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPID(PIDConfig pid) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(pid.kP(), pid.kI(), pid.kD());
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor.set(Math.signum(velocityRadPerSec));
    // pid.setSetpoint(
    //     Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
    //     ControlType.kVelocity,
    //     ClosedLoopSlot.kSlot0,
    //     ffVolts,
    //     ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void updateInputs(HopperMotorIOInputs inputs) {
    SparkUtil.clearError();

    SparkUtil.ifOk(
        motor, encoder::getPosition, value -> inputs.positionRad = Units.rotationsToRadians(value));
    SparkUtil.ifOk(
        motor,
        encoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    SparkUtil.ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = value);
    SparkUtil.ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);

    inputs.motorConnected = connectionDebouncer.calculate(!SparkUtil.hasError());
  }
}
