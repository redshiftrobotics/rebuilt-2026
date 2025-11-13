package frc.robot.subsystems.examples.template;

import frc.robot.Constants;

/** Constants for the Template subsystem. */
public class TemplateConstants {

  // Example of a constant that is not dependent on the robot
  public static final double SPEED = 0.5;

  // Example of a constant that is dependent on the robot
  public static final int CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1;
        case SIM_BOT -> 0;
        default -> 0;
      };
}
