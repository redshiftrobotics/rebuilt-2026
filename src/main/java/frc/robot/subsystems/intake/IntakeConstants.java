package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

public class IntakeConstants {

  public static final IdleMode SLAPDOWN_BRAKE_MODE = IdleMode.kCoast;
  public static final double INTAKE_WHEEL_SPEED = .5;

  public static final Rotation2d SLAPDOWN_DOWN_SETPOINT = Rotation2d.fromDegrees(0);
  public static final Rotation2d SLAPDOWN_UP_SETPOINT = Rotation2d.fromDegrees(157);

  public static final double WHEEL_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1;
        case SIM_BOT -> 1;
        default -> 1;
      };

  public static final double SLAPDOWN_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1;
        case SIM_BOT -> 1;
        default -> 1;
      };

  public static final int INTAKE_WHEEL_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  public static final int SLAPDOWN_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  public static final PIDConstants SLAPDOWN_PID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new PIDConstants(0, 0, 0);
        case SIM_BOT -> new PIDConstants(.32, 0, .14);
        default -> new PIDConstants(1, 0, 0);
      };
}
