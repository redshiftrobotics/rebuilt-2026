package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;
import frc.robot.utility.tunable.TunableNumber;

/** Constants for the Template subsystem. */
public class LauncherConstants {

  public static final Transform3d ROBOT_TO_LAUNCHER =
      new Transform3d(
          Units.inchesToMeters(-9.937105),
          0.0,
          Units.inchesToMeters(17.731846 + (4.0 / 2.0)),
          Rotation3d.kZero); // From CAD

  public static final Translation2d LAUNCHER_TO_ROBOT =
      ROBOT_TO_LAUNCHER.getTranslation().toTranslation2d().unaryMinus();

  // TODO find out real min_distance value
  public static final double MIN_DISTANCE = 0.9;
  public static final double MAX_DISTANCE = 4.9;
  public static final double PHASE_DELAY = 0.03; // estimate

  public static class ChannelConstants {
    public record ChannelConfig(int deviceId, double gearRatio, boolean inverted) {}

    public static final DCMotor motor = DCMotor.getKrakenX60(1);

    public static final double PEAK_REVERSE_VOLTAGE_PERCENTAGE = 0;

    public static final ChannelConfig LEFT_CONFIG = new ChannelConfig(3, 1, false);
    public static final ChannelConfig CENTER_CONFIG = new ChannelConfig(15, 1, false);
    public static final ChannelConfig RIGHT_CONFIG = new ChannelConfig(4, 1, true);

    public static final PIDConfig FLYWHEEL_PID =
        switch (Constants.getRobot()) {
          case REBUILT_2026 -> new PIDConfig(0.5, 2.0, 0.0);
          case SIM_BOT -> new PIDConfig(0, 0.0, 0.0);
          default -> new PIDConfig(0.0, 0.0, 0.0);
        };
    public static final FeedForwardConfigRecord FF =
        switch (Constants.getRobot()) {
          case REBUILT_2026 -> new FeedForwardConfigRecord(
              0.0, 12.0 / Units.radiansToRotations(motor.freeSpeedRadPerSec), 0.0);
          case SIM_BOT -> new FeedForwardConfigRecord(0.0, 0.019, 0.0);
          default -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        };
  }

  public static class HoodConstants {
    public static final int ACTUATOR_LEFT_ID = 0;
    public static final int ACTUATOR_RIGHT_ID = 1;

    public static final Rotation2d FIXED_LAUNCH_ANGLE = Rotation2d.fromDegrees(75);
    public static final Distance HOOD_RADIUS = Inches.of(7.414316);
    // The angle between the actuator point of attachment and the end of the hood
    public static final Rotation2d ANGLE_ADJUSTMENT = Rotation2d.fromDegrees(6.6153321);

    public static final Translation2d LAUNCHER_AXLE_TO_ACTUATOR =
        new Translation2d(Inches.of(-5.834047), Inches.of(-4.899));
    public static final Distance ACTUATOR_LENGTH_MIN = Inches.of(6.610);
    public static final Distance ACTUATOR_EXTENSION = Millimeters.of(100);
  }

  public static class LauncherMathConstants {
    public static final Distance LAUNCHER_WHEEL_RADIUS = Inches.of(2);
    public static final Mass LAUNCHER_WHEEL_MASS = Pounds.of(2.2);
    public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.0007901271);

    // TODO account for wheel MOI
    public static final MomentOfInertia TOTAL_MOI = FLYWHEEL_MOI;
    public static final Distance FUEL_RADIUS = Inches.of(3);
    public static final Mass FUEL_MASS = Pounds.of(0.5);
    public static final MomentOfInertia FUEL_MOI =
        KilogramSquareMeters.of(
            FUEL_MASS.in(Kilograms)
                * Math.pow(FUEL_RADIUS.plus(LAUNCHER_WHEEL_RADIUS).in(Meters), 2));

    // Give flywheel additional velocity to account for velocity lost in momentum transfer
    // Give flywheel double  velocity to account for spin. It rolls the ball, rather than pushing
    public static final TunableNumber LAUNCHER_VELOCITY_MULTIPLIER =
        new TunableNumber("Launcher/VelocityMultiplier", 2.3);
  }
}
