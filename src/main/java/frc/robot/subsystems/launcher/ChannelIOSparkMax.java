package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.examples.flywheel.MotorConstants;
import frc.robot.utility.records.PIDConfig;

/** Hardware implementation of the TemplateIO. */
public class ChannelIOSparkMax implements ChannelIO {
  private final String name;
  private final SparkMax motor;
  private final SparkClosedLoopController controller;

  private final RelativeEncoder encoder;

  public ChannelIOSparkMax(String name, MotorConstants constants) {
    this.name = name;

    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.voltageCompensation(12.0).smartCurrentLimit(30).idleMode(IdleMode.kCoast);
    leaderConfig.encoder.velocityConversionFactor(1.0);
    leaderConfig.inverted(constants.inverted());

    motor = new SparkMax(constants.deviceId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
    motor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(ChannelIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    controller.setSetpoint(
        velocity.in(RotationsPerSecond) * 60, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setPID(PIDConfig pid) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(pid.kP(), pid.kI(), pid.kD());
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
