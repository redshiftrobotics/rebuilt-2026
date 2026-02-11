package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/** Constants for the Template subsystem. */
public class LauncherConstants {

  public static int ACTUATOR_LEFT_ID;
  public static int ACTUATOR_RIGHT_ID;

  public static int CHANNEL_LEFT_ID;
  public static int CHANNEL_CENTER_ID;
  public static int CHANNEL_RIGHT_ID;

  public static double FLYWHEEL_KP = 1.0;
  public static double FLYWHEEL_KI = 0.0;
  public static double FLYWHEEL_KD = 0.0;

  public static Distance WHEEL_RADIUS = Inches.of(3);

  // TODO Add real hood values
  public static Rotation2d FIXED_LAUNCH_ANGLE = new Rotation2d(75);
  public static Distance HOOD_RADIUS = Inches.of(20);

  // TODO Add real actuator values
  public static Translation2d ACTUATOR_LOCATION = new Translation2d(Inches.of(-12), Inches.of(-4));
  public static Distance ACTUATOR_LENGTH_MIN = Inches.of(12);
  public static Distance ACTUATOR_LENGTH_MAX = Inches.of(24);
}
