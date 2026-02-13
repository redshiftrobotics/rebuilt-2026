package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface ChannelIO {
  @AutoLog
  public static class ChannelIOInputs {
    public double velocityRadPerSec = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ChannelIOInputs inputs) {}

  /** Run closed loop at the given speed, specifying the linear speed of the wheel edge. */
  public default void setSpeed(AngularVelocity speed) {}

  public default boolean isAtSetpoint() {
    return false;
  }

  public default void configurePID(double kP, double kI, double kD) {}
}
