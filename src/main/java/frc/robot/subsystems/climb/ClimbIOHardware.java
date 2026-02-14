package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/** Hardware implementation of the ClimbIO. */
public class ClimbIOHardware implements ClimbIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid; // TODO Do I need this?

  public ClimbIOHardware() {
    motor = new SparkMax(ClimbConstants.CAN_ID, MotorType.kBrushless);

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    // --- Configure Hardware ---

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .inverted(ClimbConstants.MOTOR_INVERTED)
        .idleMode(IdleMode.kCoast);

    motor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
