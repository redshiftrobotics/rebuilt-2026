package frc.robot.subsystems.outtake;

import static frc.robot.utility.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final TalonFX[] motors;
  private final boolean[] inverts = {false, false, true};

  public Outtake(RobotType robotType) {

    if (robotType != RobotType.REBUILT_2026) {
      motors = new TalonFX[] {};
    } else {
      motors =
          new TalonFX[] {
            new TalonFX(3), // left
            new TalonFX(4), // middle
            new TalonFX(15) // right
          };
    }

    for (int i = 0; i < motors.length; i++) {
      final TalonFX motor = motors[i];
      final TalonFXConfiguration config = new TalonFXConfiguration();

      config.TorqueCurrent.PeakForwardTorqueCurrent = 120;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -120;

      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      config.MotorOutput.Inverted =
          inverts[i] ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

      tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    }
  }

  public void setSpeed(double speed) {
    Logger.recordOutput(getName() + "/Speed", speed);

    for (int i = 0; i < motors.length; i++) {
      motors[i].set(speed);
    }
  }
}
