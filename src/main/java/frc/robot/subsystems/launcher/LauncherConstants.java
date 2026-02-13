package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Constants for the Template subsystem. */
public class LauncherConstants {

  public static int ACTUATOR_LEFT_ID;
  public static int ACTUATOR_RIGHT_ID;

  public static int CHANNEL_LEFT_ID;
  public static int CHANNEL_CENTER_ID;
  public static int CHANNEL_RIGHT_ID;

  public static double FLYWHEEL_KP = 0.92;
  public static double FLYWHEEL_KI = 0.0;
  public static double FLYWHEEL_KD = 0.0;

  // TODO Update
  public static Distance LAUNCHER_WHEEL_RADIUS = Inches.of(2);
  public static Mass LAUNCHER_WHEEL_MASS = Pounds.of(0.3);
  public static MomentOfInertia LAUNCHER_WHEEL_MOI =
      KilogramSquareMeters.of(
          LAUNCHER_WHEEL_MASS.in(Kilograms)
              * LAUNCHER_WHEEL_RADIUS.in(Meters)
              * LAUNCHER_WHEEL_RADIUS.in(Meters));

  public static Distance FLYWHEEL_RADIUS = Inches.of(2);
  public static Mass FLYWHEEL_MASS = Pounds.of(4);
  public static MomentOfInertia FLYWHEEL_MOI =
      KilogramSquareMeters.of(
          FLYWHEEL_MASS.in(Kilograms) * FLYWHEEL_RADIUS.in(Meters) * FLYWHEEL_RADIUS.in(Meters));

  public static MomentOfInertia TOTAL_MOI = LAUNCHER_WHEEL_MOI.plus(FLYWHEEL_MOI);
  public static Mass FUEL_MASS = Pounds.of(0.5);

  public static double LAUNCHER_VELOCITY_MULTIPLIER =
      2
          * (LauncherConstants.FUEL_MASS.in(Kilograms)
              + LauncherConstants.TOTAL_MOI.in(KilogramSquareMeters))
          / LauncherConstants.TOTAL_MOI.in(KilogramSquareMeters);
  // TODO Double check
  public static Distance LAUNCHER_X_OFFSET = Inches.of(-12);
  public static Distance LAUNCHER_Z_OFFSET = Inches.of(20);
  public static Distance HUB_Z_OFFSET = Feet.of(6);

  // TODO Add real hood values
  public static Rotation2d FIXED_LAUNCH_ANGLE = new Rotation2d(75);
  public static Distance HOOD_RADIUS = Inches.of(20);

  // TODO Add real actuator values
  public static Translation2d ACTUATOR_LOCATION = new Translation2d(Inches.of(-12), Inches.of(-4));
  public static Distance ACTUATOR_LENGTH_MIN = Inches.of(12);
  public static Distance ACTUATOR_EXTENSION = Inches.of(24);
}
