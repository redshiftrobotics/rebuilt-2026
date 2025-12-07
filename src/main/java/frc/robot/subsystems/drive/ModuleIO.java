package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for all swerve module hardware */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    boolean driveMotorConnected = false;
    double drivePositionRad = 0.0;
    double driveVelocityRadPerSec = 0.0;
    double driveAppliedVolts = 0.0;
    double driveSupplyCurrentAmps = 0.0;

    boolean turnMotorConnected = false;
    Rotation2d turnAbsolutePosition = new Rotation2d();
    Rotation2d turnPosition = new Rotation2d();
    double turnVelocityRadPerSec = 0.0;
    double turnAppliedVolts = 0.0;
    double turnSupplyCurrentAmps = 0.0;

    boolean turnAbsoluteEncoderConnected = false;

    double[] odometryTimestamps = new double[] {};
    double[] odometryDrivePositionsRad = new double[] {};
    Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified amount. */
  default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified amount. */
  default void setTurnOpenLoop(double output) {}

  /** Run to drive velocity setpoint with feedforward */
  default void setDriveVelocity(double velocityRadsPerSec) {}

  /** Run to turn position setpoint */
  default void setTurnPosition(double angleRads) {}

  /** Configure drive PID */
  default void setDrivePID(double kP, double kI, double kD) {}

  /** Configure turn PID */
  default void setTurnPID(double kP, double kI, double kD) {}

  /** Configure turn feedforward */
  default void setDriveFF(double kS, double kV, double kA) {}

  /** Configure turn feedforward */
  default void setTurnFF(double kS, double kV, double kA) {}

  /** Enable or disable brake mode on the drive motor. */
  default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  default void setTurnBrakeMode(boolean enable) {}

  /** Disable output to brake and turn motor */
  default void stop() {}
}
