package frc.robot.subsystems.hopper;

import frc.robot.Constants;

public class HopperConstants {
  /*
   * All constants related to the hopper (bubbler and feeder) should go here.
   * Don't make a separate file for the components.
   */

  public static final int BUBBLER_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  public static final int FEEDER_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };
}
