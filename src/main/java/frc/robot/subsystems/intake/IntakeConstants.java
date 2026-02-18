package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConfig;

public class IntakeConstants {
  public static final double INTAKE_WHEEL_SPEED = .5;
  public static final boolean INTAKE_WHEEL_INVERTED = false;
  public static final double INTAKE_WHEEL_GEAR_RATIO = 0.03125;

  public static final boolean SLAPDOWN_WHEEL_INVERTED = false;
  public static final Rotation2d SLAPDOWN_DOWN_SETPOINT = Rotation2d.fromDegrees(0);
  public static final Rotation2d SLAPDOWN_UP_SETPOINT = Rotation2d.fromDegrees(157);
  public static final Rotation2d SLAPDOWN_INCREMENT_SETPOINT = Rotation2d.fromDegrees(5);

  public static final double SLAPDOWN_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1;
        case SIM_BOT -> 1;
        default -> 1;
      };

  public static final int INTAKE_WHEEL_CAN_ID = 5;

  public static final int SLAPDOWN_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  public static final PIDConfig SLAPDOWN_PID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new PIDConfig(0, 0, 0);
        case SIM_BOT -> new PIDConfig(.32, 0, .14);
        default -> new PIDConfig(1, 0, 0);
      };
}
