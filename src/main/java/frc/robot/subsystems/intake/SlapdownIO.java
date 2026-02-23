package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utility.records.PIDConfig;
import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
  @AutoLog
  public class SlapdownIOInputs {
    public boolean motorConnected = true;
    public boolean encodersAligned = true;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double absolutePositionRad = 0.0;
    public double absoluteVelocityRadPerSec = 0.0;

    public double appliedVolts;
    public double supplyCurrentAmps;
  }

  public default void updateInputs(SlapdownIOInputsAutoLogged inputs) {}

  public default void setPID(PIDConfig config) {}

  public default void setSetpoint(Rotation2d setPoint) {}

  public default void setSavedUpSetpoint(Rotation2d setPoint) {}

  public default void setSavedDownSetpoint(Rotation2d setPoint) {}

  public default void stopMotor() {}
}
