package frc.robot.subsystems.hopper.bubbler;

import org.littletonrobotics.junction.AutoLog;

public interface BubblerIO {
  @AutoLog
  public static class BubblerIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(BubblerIOInputs inputs) {}

  /** Configure the PID constants */
  public default void configurePID(double kP, double kI, double kD) {}

  /** Stop the motor */
  public default void stop() {}

  /** Run the motor in closed-loop mode at the specified velocity */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}
}
