package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;

/** Interface for the IO layers of the Template subsystem. */
public class HoodIOFixed implements HoodIO {

  /** Updates the set of loggable inputs. */
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angleRadians = LauncherConstants.FIXED_LAUNCH_ANGLE.getRadians();
  }

  public void setLaunchAngle(Rotation2d angle) {}

  public Rotation2d getLaunchAngle() {
    return LauncherConstants.FIXED_LAUNCH_ANGLE;
  }
}
