package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

/** Hardware implementation of the ClimbIO. */
public class ClimbIOHardware implements ClimbIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid; // TODO Do I need this?
  DigitalInput leftLimitSwitch;

  public ClimbIOHardware() {

    SparkBaseConfig leaderConfig =
        new SparkMaxConfig()
            .voltageCompensation(12.0)
            .smartCurrentLimit(30)
            .inverted(ClimbConstants.MOTOR_INVERTED)
            .idleMode(IdleMode.kCoast);

    motor = new SparkMax(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
    motor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    leftLimitSwitch = new DigitalInput(ClimbConstants.SWITCH_ID);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    double rotations = encoder.getPosition() / ClimbConstants.GEAR_RATIO;
    double rpm = encoder.getVelocity() / ClimbConstants.GEAR_RATIO;

    inputs.positionRad = Units.rotationsToRadians(rotations);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rpm);

    double appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();

    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};

    inputs.climberDown = isAtBottom();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public boolean isAtBottom() {
    return leftLimitSwitch.get();
  }
}
