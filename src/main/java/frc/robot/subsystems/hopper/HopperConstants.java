package frc.robot.subsystems.hopper;

import frc.robot.Constants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConstants;

public class HopperConstants {
  /*
   * All constants related to the hopper (bubbler and feeder) should go here.
   * Don't make a separate file for the components.
   */

  // Bubbler motor CAN ID
  public static final int BUBBLER_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  // Feeder motor CAN ID
  public static final int FEEDER_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  // Hopper run mode
  // TODO: Set real speeds
  public static enum RunMode {
    STOPPED(0, 0),
    /** All hopper motors stopped */
    FUEL_STORE(30, 0),
    /** Bubbler running at low speed to push fuel to the back, feeder stopped */
    FIRING(60, 60),
    /** Bubbler running at high speed to send balls to the feeder, feeder running */
    REVERSE(-60, -60);
    /** All hopper motors running in reverse in case of jams */
    public int bubblerVelocityRadPerSec;

    public int feederVelocityRadPerSec;

    private RunMode(int bubblerVelocity, int feederVelocity) {
      bubblerVelocityRadPerSec = bubblerVelocity;
      feederVelocityRadPerSec = feederVelocity;
    }

    @Override
    public String toString() {
      switch (this) {
        case STOPPED:
          return "Stopped";
        case FUEL_STORE:
          return "Fuel Storing";
        case FIRING:
          return "Firing";
        case REVERSE:
          return "Reverse";
        default:
          return "Unknown";
      }
    }
  }

  // PID constants
  // TODO: Add real values
  public static final PIDConstants BUBBLER_PID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new PIDConstants(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(0.85, 0.0, 0.0);
        default -> new PIDConstants(1.0, 0.0, 0.0);
      };
  public static final PIDConstants FEEDER_PID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new PIDConstants(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(0.85, 0.0, 0.0);
        default -> new PIDConstants(1.0, 0.0, 0.0);
      };

  // Feedforward constants
  // TODO: Add real values
  public static final FeedForwardConfigRecord BUBBLER_FF =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        case SIM_BOT -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        default -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
      };
  public static final FeedForwardConfigRecord FEEDER_FF =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        case SIM_BOT -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        default -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
      };

  // Gear ratios
  // TODO: Add real values (thank you design team)
  public static final double BUBBLER_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1.0;
        case SIM_BOT -> 1.0;
        default -> 1.0;
      };
  public static final double FEEDER_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1.0;
        case SIM_BOT -> 1.0;
        default -> 1.0;
      };
}
