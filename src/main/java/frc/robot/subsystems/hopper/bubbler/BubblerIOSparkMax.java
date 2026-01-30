package frc.robot.subsystems.hopper.bubbler;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hopper.HopperConstants;

public class BubblerIOSparkMax implements BubblerIO {
  /* Motor */
  private final SparkMax motor;

  /* PID controller */
  private final SparkClosedLoopController pidController;

  /* Encoder */
  private final RelativeEncoder encoder;

  public BubblerIOSparkMax() {
    // Create motor
    motor = new SparkMax(HopperConstants.BUBBLER_CAN_ID, MotorType.kBrushless);

    // Get motor resources
    pidController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // Setup config object
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);

    // Apply config
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // Set setpoint
    pidController.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    // Stop
    motor.stopMotor();
  }

  @Override
  public void updateInputs(BubblerIOInputs inputs) {
    // Motor position
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / HopperConstants.BUBBLER_GEAR_RATIO);

    // Motor velocity
    inputs.velocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity() / HopperConstants.BUBBLER_GEAR_RATIO);

    // Voltage input to the motor
    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };

    // Motor output current
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }
}
