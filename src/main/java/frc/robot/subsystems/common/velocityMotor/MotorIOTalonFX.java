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

/** Module IO implementation for Talon FX drive motor controller. */
public class MotorIOTalonFX implements MotorIO {
  // Hardware objects
  private final TalonFX driveTalon;

  // Config
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final ClosedLoopOutputType outputType;

  // Break or coast mode
  private boolean driveBrakeMode = true;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);

  public MotorIOTalonFX(VelocityMotorConfig config) {
    this(config, ClosedLoopOutputType.TorqueCurrentFOC);
  }

  public MotorIOTalonFX(VelocityMotorConfig config, ClosedLoopOutputType outputType) {

    driveTalon = new TalonFX(config.deviceId());

    driveBrakeMode = config.brakeMode();
    this.outputType = outputType;

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode =
        driveBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveConfig.Slot0 = new Slot0Configs();
    driveConfig.Feedback.SensorToMechanismRatio = config.gearRatio();
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.stallCurrent();
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -config.stallCurrent();
    driveConfig.CurrentLimits.StatorCurrentLimit = config.stallCurrent();
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        config.inverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    // Update drive inputs
    inputs.driveMotorConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveCurrent.getValueAsDouble();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    if (output == 0) {
      driveTalon.stopMotor();
      return;
    }

    driveTalon.setControl(
        switch (outputType) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedfowrad) {
    if (velocityRadPerSec == 0) {
      driveTalon.stopMotor();
      return;
    }

    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
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
  public void setDriveBrakeMode(boolean enable) {
    if (this.driveBrakeMode != enable) {
      this.driveBrakeMode = enable;
      driveTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void stop() {
    driveTalon.stopMotor();
  }
}
