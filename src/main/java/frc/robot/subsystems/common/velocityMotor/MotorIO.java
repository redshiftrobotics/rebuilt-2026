package frc.robot.subsystems.common.velocityMotor;

import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for all swerve module hardware */
public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    boolean driveMotorConnected = true;

    double drivePositionRad = 0.0;
    double driveVelocityRadPerSec = 0.0;
    double driveAppliedVolts = 0.0;
    double driveSupplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(MotorIOInputs inputs);

  /** Run the drive motor at the specified amount. */
  public void setDriveOpenLoop(double output);

  /** Run to drive velocity setpoint */
  public default void setDriveVelocity(double velocityRadsPerSec) {
    setDriveVelocity(velocityRadsPerSec, 0);
  }

  /** Run to drive velocity setpoint with feedforward */
  public void setDriveVelocity(double velocityRadsPerSec, double feedforward);

  /** Configure drive PID */
  public void setDrivePID(double kP, double kI, double kD);

  /** Enable or disable brake mode on the drive motor. */
  public void setDriveBrakeMode(boolean enable);

  /** Disable output to brake and turn motor */
  public void stop();
}
