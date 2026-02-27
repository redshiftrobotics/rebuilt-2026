package frc.robot.subsystems.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Dead simple interpolation shot calculator */
public class InterpolationShotCalculator {

  private static InterpolatingDoubleTreeMap speeds = new InterpolatingDoubleTreeMap();

  static {
    // Map distances to launcher wheel speeds.
    speeds.put(1.0, 100.0);
    speeds.put(2.0, 200.0);
    speeds.put(3.0, 300.0);
  }

  public static double calculateWheelVelocity(double distance) {
    return speeds.get(distance);
  }
}
