// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.common.velocityMotor;

import static frc.robot.utility.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Motor IO implementation for Talon FX motor controller. */
public class MotorIOTalonFX implements MotorIO {

  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> veclity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final ClosedLoopOutputType outputType;

  private boolean brakeMode = true;

  // Connection debouncers
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public MotorIOTalonFX(VelocityMotorConstants constants) {
    this(constants, ClosedLoopOutputType.TorqueCurrentFOC);
  }

  public MotorIOTalonFX(VelocityMotorConstants constants, ClosedLoopOutputType outputType) {

    motor = new TalonFX(constants.deviceId());

    brakeMode = constants.brakeMode();
    this.outputType = outputType;

    config.MotorOutput.NeutralMode =
        brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Slot0 = new Slot0Configs();
    config.Feedback.SensorToMechanismRatio = constants.gearRatio();
    config.TorqueCurrent.PeakForwardTorqueCurrent = constants.stallCurrent();
    config.TorqueCurrent.PeakReverseTorqueCurrent = -constants.stallCurrent();
    config.CurrentLimits.StatorCurrentLimit = constants.stallCurrent();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        constants.inverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));

    position = motor.getPosition();
    veclity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, veclity, appliedVolts, current);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(position, veclity, appliedVolts, current);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(veclity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = current.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    if (output == 0) {
      motor.stopMotor();
      return;
    }

    motor.setControl(
        switch (outputType) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedfowrad) {
    if (velocityRadPerSec == 0) {
      motor.stopMotor();
      return;
    }

    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    motor.setControl(
        switch (outputType) {
          case Voltage -> velocityVoltageRequest
              .withVelocity(velocityRotPerSec)
              .withFeedForward(feedfowrad);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest
              .withVelocity(velocityRotPerSec)
              .withFeedForward(feedfowrad);
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
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
