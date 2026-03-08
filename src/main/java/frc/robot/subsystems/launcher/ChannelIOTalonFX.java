package frc.robot.subsystems.launcher;

import static frc.robot.utility.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.launcher.LauncherConstants.ChannelConstants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

/** Channel IO implementation for TalonFX motor controller */
public class ChannelIOTalonFX implements ChannelIO {

  private final String name;
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Double> dutyCycle;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ChannelIOTalonFX(String name, ChannelConstants constants) {
    this.name = name;
    motor = new TalonFX(constants.deviceId());

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0 = new Slot0Configs();
    config.Feedback.SensorToMechanismRatio = constants.gearRatio();

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        constants.inverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Audio.BeepOnBoot = Constants.TALON_BEEP_ON_BOOT;
    config.Audio.BeepOnConfig = Constants.TALON_BEEP_ON_CONFIG;

    config.Voltage.PeakReverseVoltage *= LauncherConstants.PEAK_REVERSE_VOLTAGE_PERCENTAGE;

    pushConfig();

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
    dutyCycle = motor.getDutyCycle();

    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(ChannelIOInputs inputs) {
    StatusCode status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current, dutyCycle);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = current.getValueAsDouble();
    inputs.appliedDutyCycle = dutyCycle.getValueAsDouble();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setVelocity(double radPerSec, double arbFeedforward) {
    double rotationsPerSec = Units.radiansToRotations(radPerSec);
    motor.setControl(
        velocityVoltageRequest.withVelocity(rotationsPerSec).withFeedForward(arbFeedforward));
  }

  @Override
  public void setPID(PIDConfig pid) {
    SmartDashboard.putString("Launcher PID Debug", pid.toString());
    config.Slot0.kP = pid.kP();
    config.Slot0.kI = pid.kI();
    config.Slot0.kD = pid.kD();
    pushConfig();
  }

  @Override
  public void setFF(FeedForwardConfigRecord ffConfig) {
    SmartDashboard.putString("Launcher FF Debug", ffConfig.toString());
    config.Slot0.kS = ffConfig.kS();
    config.Slot0.kV = ffConfig.kV();
    config.Slot0.kA = ffConfig.kA();
    pushConfig();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  private void pushConfig() {
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }
}
