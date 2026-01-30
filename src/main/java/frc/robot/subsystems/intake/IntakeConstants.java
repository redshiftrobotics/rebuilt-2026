package frc.robot.subsystems.intake;

import frc.robot.Constants;

/** Constants for the Template subsystem. */
public class IntakeConstants {

  // Example of a constant that is not dependent on the robot
  public static final double SPEED = 0.5;
  // Make the following constants real values
  public static final double JKgMetersSquared = 1;
  public static final double gearing = 1; /// maybe change

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
