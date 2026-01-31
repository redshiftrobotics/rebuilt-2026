package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class IntakeConstants {

  public static final IdleMode SlapdownBrakeMode = IdleMode.kCoast;

  public static final int WHEEL_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  public static final int SLAPDOWN_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  public static final int WHEEL_CAN_ID =
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
}
