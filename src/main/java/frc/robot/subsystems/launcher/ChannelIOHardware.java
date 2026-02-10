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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

/** Hardware implementation of the TemplateIO. */
public class ChannelIOHardware implements ChannelIO {
  private final SparkMax motor;
  private final SparkClosedLoopController controller;

  private final RelativeEncoder encoder;

  public ChannelIOHardware(int motorID) {

    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.voltageCompensation(12.0).smartCurrentLimit(30).idleMode(IdleMode.kCoast);
    leaderConfig.encoder.velocityConversionFactor(1.0);

    motor = new SparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    motor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ChannelIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    controller.setSetpoint(
        speed.in(RotationsPerSecond) * 60, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
