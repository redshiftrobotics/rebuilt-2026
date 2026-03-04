package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

class LauncherControlAutomatic {

  public static Rotation2d calculatePitch(Distance distance) {
    // Formula from experimentation
    return Rotation2d.fromDegrees(75.0 - 15.0 * Math.tanh(2.0 * distance.in(Feet) / 25.0));
  }
}
