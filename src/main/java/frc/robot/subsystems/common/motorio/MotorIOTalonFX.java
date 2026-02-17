package frc.robot.subsystems.common.motorio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utility.records.PIDConfig;

public class MotorIOTalonFX implements MotorIO {
  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;

  private final StatusSignal<Current> current;

  private final double gearRatio;

  public MotorIOTalonFX(int motorId, boolean inverted, double gearRatio) {
    motor = new TalonFX(motorId);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getSupplyCurrent();

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();

    this.gearRatio = gearRatio;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / gearRatio;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / gearRatio;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTargetPosition(Rotation2d targetPosition) {
    motor.setControl(new PositionVoltage(targetPosition.getRotations()));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double Kp, double Ki, double Kd) {
    var config = new Slot0Configs();
    config.kP = Kp;
    config.kI = Ki;
    config.kD = Kd;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void configurePID(PIDConfig config) {
    var motorConfig = new Slot0Configs();
    motorConfig.kP = config.kP();
    motorConfig.kI = config.kI();
    motorConfig.kD = config.kD();

    motor.getConfigurator().apply(motorConfig);
  }
}
