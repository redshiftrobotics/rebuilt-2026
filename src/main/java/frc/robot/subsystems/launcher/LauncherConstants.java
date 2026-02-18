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
import frc.robot.utility.tunable.TunableNumber;

/** Constants for the Template subsystem. */
public class LauncherConstants {
  // TODO Add real IDs
  public static final int ACTUATOR_LEFT_ID = 1;
  public static final int ACTUATOR_RIGHT_ID = 2;

  public static final int CHANNEL_LEFT_ID = 1;
  public static final int CHANNEL_CENTER_ID = 2;
  public static final int CHANNEL_RIGHT_ID = 3;

  public static final double FLYWHEEL_KP = 0.92;
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0;

  // TODO Update
  public static final Distance LAUNCHER_WHEEL_RADIUS = Inches.of(2);
  public static final Mass LAUNCHER_WHEEL_MASS = Pounds.of(0.3);
  public static final MomentOfInertia LAUNCHER_WHEEL_MOI =
      KilogramSquareMeters.of(
          LAUNCHER_WHEEL_MASS.in(Kilograms)
              * LAUNCHER_WHEEL_RADIUS.in(Meters)
              * LAUNCHER_WHEEL_RADIUS.in(Meters));

  public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
  public static final Mass FLYWHEEL_MASS = Pounds.of(4);
  public static final MomentOfInertia FLYWHEEL_MOI =
      KilogramSquareMeters.of(
          FLYWHEEL_MASS.in(Kilograms) * FLYWHEEL_RADIUS.in(Meters) * FLYWHEEL_RADIUS.in(Meters));

  public static final MomentOfInertia TOTAL_MOI = LAUNCHER_WHEEL_MOI.plus(FLYWHEEL_MOI);
  public static final Mass FUEL_MASS = Pounds.of(0.5);

  public static final TunableNumber LAUNCHER_TUNING_PARAMETER = 
      new TunableNumber("Launcher/VelocityMultiplier", 1);

  // Give flywheel additional velocity to account for velocity lost in momentum transfer
  // Give flywheel double  velocity to account for spin. It rolls the ball, rather than pushing
  public static final double LAUNCHER_VELOCITY_MULTIPLIER =
      2
          * (LauncherConstants.FUEL_MASS.in(Kilograms)
              + (LauncherConstants.TOTAL_MOI.in(KilogramSquareMeters))
              + LauncherConstants.FUEL_MASS.in(Kilograms))
          / LauncherConstants.TOTAL_MOI.in(KilogramSquareMeters);
  // TODO Double check
  public static final Distance LAUNCHER_X_OFFSET = Inches.of(-12);
  public static final Distance LAUNCHER_Z_OFFSET = Inches.of(20);
  public static final Distance HUB_Z_OFFSET = Feet.of(6);

  public static final HoodType HOOD_TYPE = HoodType.FIXED;
  // TODO Add real hood values
  public static final Rotation2d FIXED_LAUNCH_ANGLE = new Rotation2d(75);
  public static final Distance HOOD_RADIUS = Inches.of(20);

  // TODO Add real actuator values
  public static final Translation2d ACTUATOR_LOCATION = new Translation2d(Inches.of(-12), Inches.of(-4));
  public static final Distance ACTUATOR_LENGTH_MIN = Inches.of(12);
  public static final Distance ACTUATOR_EXTENSION = Inches.of(24);

  public static final Distance MIN_DISTANCE = Feet.of(5);
  public static final Distance MAX_DISTANCE = Feet.of(25);
}

enum HoodType {
  FIXED,
  ACTUATOR,
}
