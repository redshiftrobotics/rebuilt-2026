package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.utility.records.FeedForwardConstants;
import frc.robot.utility.records.PIDConstants;

public class ModuleConstants {

  // --- Module Config ---

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {
    public ModuleConfig(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            config) {
      this(
          config.DriveMotorId,
          config.SteerMotorId,
          config.EncoderId,
          Rotation2d.fromRotations(config.EncoderOffset),
          config.SteerMotorInverted);
    }
  }

  public static final ModuleConfig FRONT_LEFT_MODULE_CONFIG;
  public static final ModuleConfig FRONT_RIGHT_MODULE_CONFIG;
  public static final ModuleConfig BACK_LEFT_MODULE_CONFIG;
  public static final ModuleConfig BACK_RIGHT_MODULE_CONFIG;

  static {
    switch (Constants.getRobot()) {
      case PRESEASON_2026:
        // DO NOT USE THESE CONSTANTS, USE TUNER CONSTANTS DIRECTLY INSTEAD, HERE FOR REFERENCE
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(TunerConstants.FrontLeft);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(TunerConstants.FrontRight);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(TunerConstants.BackLeft);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(TunerConstants.BackRight);
        break;

      case REEFSCAPE_2025:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(19, 18, 37, Rotation2d.fromRotations(-0.596435546875), true);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(2, 1, 36, Rotation2d.fromRotations(-0.96826171875), true);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(10, 11, 39, Rotation2d.fromRotations(-0.477294921875), true);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 38, Rotation2d.fromRotations(-0.530517578125), true);
        break;

      case CHASSIS_CANNON:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(19, 18, 39, Rotation2d.fromRotations(-0.186279296875), true);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(2, 1, 37, Rotation2d.fromRotations(-0.677490234375 + 0.5), true);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(11, 10, 36, Rotation2d.fromRotations(-0.8603515625), true);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 38, Rotation2d.fromRotations(-0.065185546875 + 0.5), true);
        break;

      case WOOD_BOT_2026:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        break;

      default:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.kZero, false);
        break;
    }
  }

  // --- Module Warnings ---

  public static final double DISCONNECTED_MOTOR_WARNING_THRESHOLD_SECONDS = 0.5;
  public static final double DISCONNECTED_ENCODER_WARNING_THRESHOLD_SECONDS = 0.2;

  // --- Module Constants ---

  public static final DCMotor DRIVE_MOTOR;
  public static final FeedForwardConstants DRIVE_FEED_FORWARD;
  public static final PIDConstants DRIVE_FEEDBACK;
  public static final double DRIVE_MOTOR_CURRENT_LIMIT;
  public static final double DRIVE_REDUCTION;

  public static final DCMotor TURN_MOTOR;
  public static final PIDConstants TURN_FEEDBACK;
  public static final FeedForwardConstants TURN_FEED_FORWARD;
  public static final double TURN_MOTOR_CURRENT_LIMIT;
  public static final double TURN_REDUCTION;

  public static final double WHEEL_RADIUS =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> TunerConstants.FrontLeft.WheelRadius;
        default -> Units.inchesToMeters(2.000);
      };

  static {
    switch (Constants.getRobot()) {
      case PRESEASON_2026:
        DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
        DRIVE_FEEDBACK = new PIDConstants(0.1, 0.0, 0.0);
        DRIVE_FEED_FORWARD = new FeedForwardConstants(0.0, 0.0, 0.0);
        DRIVE_MOTOR_CURRENT_LIMIT = TunerConstants.FrontLeft.SlipCurrent;
        DRIVE_REDUCTION = TunerConstants.FrontLeft.DriveMotorGearRatio;

        TURN_MOTOR = DCMotor.getKrakenX60Foc(1);
        TURN_FEEDBACK = new PIDConstants(75, 0.0, 2.7);
        TURN_FEED_FORWARD = new FeedForwardConstants(2.0, 16.0, 0.0);
        TURN_MOTOR_CURRENT_LIMIT = TunerConstants.FrontLeft.SlipCurrent;
        TURN_REDUCTION = TunerConstants.FrontLeft.SteerMotorGearRatio;
        break;

      case SIM_BOT:
        DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
        DRIVE_FEEDBACK = new PIDConstants(0.05, 0.0, 0.0);
        DRIVE_FEED_FORWARD = new FeedForwardConstants(0.0, 0.144886, 0.0);
        DRIVE_MOTOR_CURRENT_LIMIT = TunerConstants.FrontLeft.SlipCurrent;
        DRIVE_REDUCTION = Mk4iReductions.L3.reduction;

        TURN_MOTOR = DCMotor.getKrakenX60Foc(1);
        TURN_FEEDBACK = new PIDConstants(8, 0, 0);
        TURN_FEED_FORWARD = new FeedForwardConstants(0.0, 0.0, 0.0);
        TURN_MOTOR_CURRENT_LIMIT = 800; // No limit
        TURN_REDUCTION = TunerConstants.FrontLeft.SteerMotorGearRatio;
        break;

      case CHASSIS_CANNON:
      case REEFSCAPE_2025:
      default:
        DRIVE_MOTOR = DCMotor.getNEO(1);
        DRIVE_FEEDBACK = new PIDConstants(0.0001, 0.0, 0.0);
        DRIVE_FEED_FORWARD = new FeedForwardConstants(0.2, 4.35, 0);
        DRIVE_MOTOR_CURRENT_LIMIT = 50;
        DRIVE_REDUCTION = Mk4iReductions.L3.reduction;

        TURN_MOTOR = DCMotor.getNEO(1);
        TURN_FEEDBACK = new PIDConstants(10, 0.0, 0.0);
        TURN_FEED_FORWARD = new FeedForwardConstants(0.0, 0.0, 0.0);
        TURN_MOTOR_CURRENT_LIMIT = 20;
        TURN_REDUCTION = Mk4iReductions.TURN_REDUCTION;
        break;
    }
  }

  public static final double TURN_ALIGNMENT_TOLERANCE_DEGREES = 1;

  // --- Module reductions ---

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  public enum Mk4iReductions {
    // Note: Mk4i turn motors are inverted!
    L1(19.0 / 25.0),
    L2(17.0 / 27.0),
    L3(16.0 / 28.0);

    public static final double TURN_REDUCTION = (150.0 / 7.0);

    public final double reduction;

    Mk4iReductions(double adjustableRatio) {
      this.reduction = (50.0 / 14.0) * adjustableRatio * (45.0 / 15.0);
    }
  }

  // https://www.swervedrivespecialties.com/products/mk5n-swerve-module
  public enum Mk5nReductions {
    L1(12.0),
    L2(14.0),
    L3(16.0);

    public static final double TURN_REDUCTION = (287.0 / 11.0);

    public final double reduction;

    Mk5nReductions(double adjustableGearTeeth) {
      this.reduction = (54.0 / adjustableGearTeeth) * (25.0 / 32.0) * (30.0 / 15.0);
    }
  }
}
