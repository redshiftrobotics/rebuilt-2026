package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelIO {
  @AutoLog
  public class IntakeWheelIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVolts;
    public double supplyCurrentAmps;

    public boolean motorConnected = true;
  }

  public default void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }

  public default void stop() {}
}
