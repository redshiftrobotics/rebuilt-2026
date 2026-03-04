package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Dead simple interpolation shot calculator */
public class LauncherControlInterpolation {

  private static final InterpolatingDoubleTreeMap fixedHoodSpeeds = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodPitchDegrees = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap actuatingHoodSpeeds = new InterpolatingDoubleTreeMap();

  static {
    // Map distances (meters) to launcher wheel speeds.
    fixedHoodSpeeds.put(1.0, 100.0);
    fixedHoodSpeeds.put(2.0, 200.0);
    fixedHoodSpeeds.put(3.0, 300.0);

    hoodPitchDegrees.put(1.0, 75.0);
    hoodPitchDegrees.put(2.0, 70.0);
    hoodPitchDegrees.put(3.0, 65.0);
    hoodPitchDegrees.put(4.0, 60.0);

    actuatingHoodSpeeds.put(1.0, 100.0);
    actuatingHoodSpeeds.put(2.0, 175.0);
    actuatingHoodSpeeds.put(3.0, 250.0);
  }

  public static double calculateVelocity(double distance) {
    return fixedHoodSpeeds.get(distance);
  }

  public static Rotation2d calculateHoodPitch(double distance) {
    return Rotation2d.fromDegrees(hoodPitchDegrees.get(distance));
  }

  public static double calculateVelocityAdjustableHood(double distance) {
    return actuatingHoodSpeeds.get(distance);
  }
}
