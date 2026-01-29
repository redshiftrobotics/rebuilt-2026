package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeConstants {

  public static final double SPEED = 0.5;

  public static final int CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1;
        case SIM_BOT -> 0;
        default -> 0;
      };
}
