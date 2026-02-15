package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double angleRadians;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(HoodIOInputs inputs);

  public void setLaunchAngle(Rotation2d angle);

  public Rotation2d getLaunchAngle();
}
