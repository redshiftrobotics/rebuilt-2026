package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.utility.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.examples.flywheel.MotorConstants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

/** Hardware implementation of the TemplateIO. */
public class ChannelIOTalonFX implements ChannelIO {
  private final String name;
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Double> dutyCycle;

  private boolean pushedConfigFault = false;

  public enum OutputType {
    Voltage,
    TorqueCurrentFOC
  }

  private final OutputType outputType;

  private boolean brakeMode = true;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ChannelIOTalonFX(String name, MotorConstants constants) {
    this(name, constants, OutputType.TorqueCurrentFOC);
  }

  public ChannelIOTalonFX(String name, MotorConstants constants, OutputType outputType) {
    this.name = name;
    motor = new TalonFX(constants.deviceId());
    brakeMode = constants.brakeMode();
    this.outputType = outputType;

    final double peakReverse = 0;

    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Slot0 = new Slot0Configs();
    config.Feedback.SensorToMechanismRatio = constants.gearRatio();
    config.TorqueCurrent.PeakForwardTorqueCurrent = constants.stallCurrent();
    config.TorqueCurrent.PeakReverseTorqueCurrent = constants.stallCurrent() * peakReverse;
    config.CurrentLimits.StatorCurrentLimit = constants.stallCurrent();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        constants.inverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // config.Audio.BeepOnBoot = Constants.TALON_BEEP_ON_BOOT;
    // config.Audio.BeepOnConfig = Constants.TALON_BEEP_ON_CONFIG;

    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0 * peakReverse;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 1;

    pushConfig();

    position = motor.getPosition();
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
    StatusCode status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, dutyCycle);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = current.getValueAsDouble();
    inputs.appliedDutyCycle = dutyCycle.getValueAsDouble();
    inputs.pushedConfigFault = pushedConfigFault;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(
        switch (outputType) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setVelocity(AngularVelocity velocity, double arbFeedforward) {
    double velocityRotPerSec = velocity.in(RotationsPerSecond);
    SmartDashboard.putString(
        "Flywheel Debug", "RadPerSec " + String.valueOf(velocity.in(RotationsPerSecond)));
    motor.setControl(
        switch (outputType) {
          case Voltage -> velocityVoltageRequest
              .withVelocity(velocityRotPerSec)
              .withFeedForward(arbFeedforward);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest
              .withVelocity(velocityRotPerSec)
              .withFeedForward(arbFeedforward);
        });
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (this.brakeMode != enable) {
      this.brakeMode = enable;
      motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
  }

  @Override
  public void setPID(PIDConfig pid) {
    SmartDashboard.putString("Launcher Channel Debug", pid.toString());
    config.Slot0.kP = pid.kP();
    config.Slot0.kI = pid.kI();
    config.Slot0.kD = pid.kD();
    pushConfig();
  }

  @Override
  public void setFF(FeedForwardConfigRecord ffConfig) {
    SmartDashboard.putString("Flywheel Debug", ffConfig.toString());
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
    boolean isOk = tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    this.pushedConfigFault = !isOk;
  }
}
