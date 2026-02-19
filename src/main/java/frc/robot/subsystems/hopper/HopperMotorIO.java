package frc.robot.subsystems.hopper;

import frc.robot.utility.records.PIDConfig;
import org.littletonrobotics.junction.AutoLog;

public interface HopperMotorIO {
  @AutoLog
  public static class HopperMotorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVolts;
    public double supplyCurrentAmps;

    public boolean motorConnected = true;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(HopperMotorIOInputs inputs) {}

  /** Configure the PID constants */
  public default void setPID(PIDConfig pid) {}

  /** Stop the motor */
  public default void stop() {}

  /** Run the motor in closed-loop mode at the specified velocity */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}
}
