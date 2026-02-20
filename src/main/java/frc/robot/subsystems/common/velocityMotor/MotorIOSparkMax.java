package frc.robot.subsystems.common.velocityMotor;

import static frc.robot.utility.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.SparkUtil;

/** Module IO implementation for SparkMax drive motor controller */
public class MotorIOSparkMax implements MotorIO {

  private final SparkMax driveSpark;
  private final RelativeEncoder driveRelativeEncoder;
  private final SparkClosedLoopController driveFeedback;

  private boolean driveBreakMode = true;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);

  public MotorIOSparkMax(VelocityMotorConfig config) {

    driveSpark = new SparkMax(config.deviceId(), MotorType.kBrushless);
    driveRelativeEncoder = driveSpark.getEncoder();
    driveFeedback = driveSpark.getClosedLoopController();

    driveBreakMode = config.brakeMode();

    // Configure drive motor
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(driveBreakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit((int) config.stallCurrent())
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(config.gearRatio())
        .velocityConversionFactor(config.gearRatio());
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveRelativeEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {

    // --- Drive ---
    SparkUtil.clearError();
    ifOk(
        driveSpark,
        driveRelativeEncoder::getPosition,
        value -> inputs.drivePositionRad = Units.rotationsToRadians(value));
    ifOk(
        driveSpark,
        driveRelativeEncoder::getVelocity,
        value -> inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    ifOk(
        driveSpark,
        () -> driveSpark.getAppliedOutput() * driveSpark.getBusVoltage(),
        value -> inputs.driveAppliedVolts = value);
    ifOk(driveSpark, driveSpark::getOutputCurrent, value -> inputs.driveSupplyCurrentAmps = value);
    inputs.driveMotorConnected = driveConnectedDebounce.calculate(!SparkUtil.hasError());
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadsPerSec, double feedforward) {
    driveFeedback.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadsPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.closedLoop.pid(kP, kI, kD);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    if (driveBreakMode != enable) {
      driveBreakMode = enable;
      SparkMaxConfig driveConfig = new SparkMaxConfig();
      driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
      tryUntilOk(
          driveSpark,
          5,
          () ->
              driveSpark.configure(
                  driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
  }

  @Override
  public void stop() {
    driveSpark.stopMotor();
  }
}
