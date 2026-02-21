package frc.robot.subsystems.common.motorio;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.records.PIDConfig;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "SparkMax" with
 * "CANSparkFlex".
 */
public class MotorIOSparkMax implements MotorIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;
  private final double gearRatio;

  public MotorIOSparkMax(int motorId, boolean inverted, double gearRatio) {

    // --- Save config ---
    motor = new SparkMax(motorId, MotorType.kBrushless);

    // --- Set up leader controller ---
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    this.gearRatio = gearRatio;

    // --- Configure Hardware ---

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .inverted(inverted)
        .idleMode(IdleMode.kCoast);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / gearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / gearRatio);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setTargetPosition(Rotation2d targetPosition) {
    pid.setSetpoint(
        targetPosition.getRotations() * gearRatio,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        0,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configurePID(PIDConfig config) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pid(config.kP(), config.kI(), config.kD());
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
