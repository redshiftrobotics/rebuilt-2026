package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.utility.records.PIDConstants;

/**
 * Constants for drivetrain/chassis. All constants should be in meters and radians (m/s, m/s^2,
 * rad/s, rad/s^2). Switch expressions must cover all cases.
 */
public class DriveConstants {

  // --- Drive Config ---

  public record DriveConfig(
      Translation2d trackCornerToCorner,
      Translation2d bumperCornerToCorner,
      double maxLinearVelocity,
      double maxLinearAcceleration) {
    public double driveBaseRadius() {
      return trackCornerToCorner.getNorm() / 2;
    }

    public double maxAngularVelocity() {
      return maxLinearVelocity() / driveBaseRadius();
    }

    public double maxAngularAcceleration() {
      return maxLinearAcceleration() / driveBaseRadius();
    }

    public Constraints getLinearConstraints() {
      return new Constraints(maxLinearVelocity(), maxLinearAcceleration());
    }

    public Constraints getAngularConstraints() {
      return new Constraints(maxAngularVelocity(), maxAngularAcceleration());
    }

    public PathConstraints getPathConstraints() {
      return getPathConstraints(1);
    }

    public PathConstraints getPathConstraints(double speedMultiplier) {
      return new PathConstraints(
          maxLinearVelocity() * speedMultiplier,
          maxLinearAcceleration(),
          maxAngularVelocity() * speedMultiplier,
          maxAngularAcceleration());
    }
  }

  public static final DriveConfig DRIVE_CONFIG =
      switch (Constants.getRobot()) {
        case PHOENIX_TUNER_X, SIM_BOT -> new DriveConfig(
            new Translation2d(
                TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(1, 1),
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            22.0);
        case CHASSIS_2025 -> new DriveConfig(
            new Translation2d(Units.inchesToMeters(22.729228), Units.inchesToMeters(22.729228)),
            new Translation2d(Units.inchesToMeters(35), Units.inchesToMeters(35)),
            7.05968,
            14.5);
        case CHASSIS_CANNON -> new DriveConfig(
            new Translation2d(Units.inchesToMeters(22.729226), Units.inchesToMeters(22.729226)),
            new Translation2d(Units.inchesToMeters(25.729226), Units.inchesToMeters(25.729226)),
            5.05968,
            14.5);
      };

  // --- Module Offsets ---
  private static final double TRACK_CENTER_X = DRIVE_CONFIG.trackCornerToCorner().getX() / 2;
  private static final double TRACK_CENTER_Y = DRIVE_CONFIG.trackCornerToCorner().getY() / 2;

  public static final Translation2d FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(+TRACK_CENTER_X, +TRACK_CENTER_Y);
  public static final Translation2d FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(+TRACK_CENTER_X, -TRACK_CENTER_Y);
  public static final Translation2d BACK_LEFT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(-TRACK_CENTER_X, +TRACK_CENTER_Y);
  public static final Translation2d BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(-TRACK_CENTER_X, -TRACK_CENTER_Y);

  // --- Gyro Config ---

  public static final int GYRO_CAN_ID =
      switch (Constants.getRobot()) {
        case CHASSIS_2025 -> 40;
        case CHASSIS_CANNON -> 40;
        case PHOENIX_TUNER_X -> TunerConstants.DrivetrainConstants.Pigeon2Id;
        default -> -1;
      };

  // --- Pathplanner Config ---

  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final Translation2d[] MODULE_TRANSLATION = {
    DriveConstants.FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER,
    DriveConstants.FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER,
    DriveConstants.BACK_LEFT_MODULE_DISTANCE_FROM_CENTER,
    DriveConstants.BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER
  };

  // --- Odometry Frequency ---

  public static final double ODOMETRY_FREQUENCY_HERTZ =
      switch (Constants.getRobot()) {
        case SIM_BOT -> 50.0;
        case PHOENIX_TUNER_X -> new CANBus(TunerConstants.DrivetrainConstants.CANBusName)
                .isNetworkFD()
            ? 250.0
            : 100.0;
        default -> 100.0;
      };

  // --- Movement Controller Config ---

  public static final PIDConstants TRANSLATION_CONTROLLER_CONSTANTS_TRAJECTORY =
      new PIDConstants(5.0, 0.0, 0.0);
  public static final PIDConstants ROTATION_CONTROLLER_CONSTANTS_TRAJECTORY =
      new PIDConstants(5.0, 0, 0.4);

  public static final PIDConstants TRANSLATION_CONTROLLER_CONSTANTS =
      new PIDConstants(5.0, 0.0, 0.0);
  public static final PIDConstants ROTATION_CONTROLLER_CONSTANTS = new PIDConstants(8, 0.0, 0.0);
  public static final double TRANSLATION_TOLERANCE = Units.inchesToMeters(0.5);
  public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(1);

  // --- Heading Controller Config ---

  public record HeadingControllerConfig(PIDConstants pid, double toleranceRadians) {}

  public static final HeadingControllerConfig HEADING_CONTROLLER_CONFIG =
      new HeadingControllerConfig(ROTATION_CONTROLLER_CONSTANTS, Units.degreesToRadians(1));
}
