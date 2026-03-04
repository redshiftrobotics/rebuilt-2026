package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;
import frc.robot.utility.tunable.TunableNumber;

/** Constants for the Template subsystem. */
public class LauncherConstants {

  public static final int ACTUATOR_LEFT_ID = 1;
  public static final int ACTUATOR_RIGHT_ID = 2;

  public record ChannelConstants(int deviceId, double gearRatio, boolean inverted) {}

  public static final ChannelConstants LEFT_CONSTANTS = new ChannelConstants(3, 1, false);
  public static final ChannelConstants CENTER_CONSTANTS = new ChannelConstants(4, 1, false);
  public static final ChannelConstants RIGHT_CONSTANTS = new ChannelConstants(15, 1, true);

  public static final PIDConfig FLYWHEEL_PID =
      switch (Constants.getRobot()) {
        case REBUILT_2026 -> new PIDConfig(0.15, 0.0, 0.0);
        case SIM_BOT -> new PIDConfig(0.15, 0.0, 0.0);
        default -> new PIDConfig(0.0, 0.0, 0.0);
      };
  public static final FeedForwardConfigRecord FF =
      switch (Constants.getRobot()) {
        case REBUILT_2026 -> new FeedForwardConfigRecord(0.0, 0.019, 0.0);
        case SIM_BOT -> new FeedForwardConfigRecord(0.0, 0.019, 0.0);
        default -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
      };

  // Math for calculating flywheel velocity based on desired ball velocity, using conservation of
  // energy and momentum
  public static final Distance LAUNCHER_WHEEL_RADIUS = Inches.of(2);

  // TODO (may be innaccurate)
  public static final Mass LAUNCHER_WHEEL_MASS = Pounds.of(2.2);

  public static final MomentOfInertia LAUNCHER_WHEEL_MOI =
      KilogramSquareMeters.of(
          0.5
              * LAUNCHER_WHEEL_MASS.in(Kilograms)
              * LAUNCHER_WHEEL_RADIUS.in(Meters)
              * LAUNCHER_WHEEL_RADIUS.in(Meters));

  // public static final Distance FLYWHEEL_RADIUS = Inches.of(0);
  // public static final Mass FLYWHEEL_MASS = Pounds.of(0);
  public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.0007901271);

  // TODO waiting on flywheel
  public static final MomentOfInertia TOTAL_MOI = LAUNCHER_WHEEL_MOI;
  public static final Distance FUEL_RADIUS = Inches.of(3);
  public static final Mass FUEL_MASS = Pounds.of(0.5);
  public static final MomentOfInertia FUEL_MOI =
      KilogramSquareMeters.of(
          FUEL_MASS.in(Kilograms)
              * Math.pow(FUEL_RADIUS.plus(LAUNCHER_WHEEL_RADIUS).in(Meters), 2));

  // Give flywheel additional velocity to account for velocity lost in momentum transfer
  // Give flywheel double  velocity to account for spin. It rolls the ball, rather than pushing
  public static final TunableNumber LAUNCHER_VELOCITY_MULTIPLIER =
      new TunableNumber("Launcher/VelocityMultiplier", 2.2);

  public static final Distance LAUNCHER_X_OFFSET = Inches.of(-12);
  public static final Distance LAUNCHER_Z_OFFSET = Inches.of(20);
  public static final Distance HUB_Z_OFFSET = Feet.of(6);

  public static final HoodType HOOD_TYPE = HoodType.FIXED;
  public static final Rotation2d FIXED_LAUNCH_ANGLE = Rotation2d.fromDegrees(75);
  public static final Distance HOOD_RADIUS = Inches.of(7.4144);

  public static final Translation2d ACTUATOR_LOCATION =
      new Translation2d(Inches.of(-5.709), Inches.of(-4.784));
  public static final Distance ACTUATOR_LENGTH_MIN = Inches.of(6.610);
  public static final Distance ACTUATOR_EXTENSION = Millimeters.of(100);

  public static final Distance MIN_DISTANCE = Feet.of(5);
  public static final Distance MAX_DISTANCE = Feet.of(25);
}

enum HoodType {
  FIXED,
  ACTUATOR,
}
