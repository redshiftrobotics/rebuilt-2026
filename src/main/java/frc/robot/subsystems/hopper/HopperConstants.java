package frc.robot.subsystems.hopper;

import frc.robot.Constants;
import frc.robot.utility.records.FeedForwardConfigRecord;
import frc.robot.utility.records.PIDConfig;

public class HopperConstants {
  /*
   * All constants related to the hopper (FEEDER and LIFTER) should go here.
   * Don't make a separate file for the components.
   */

  // FEEDER motor CAN ID
  public static final int FEEDER_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  // LIFTER motor CAN ID
  public static final int LIFTER_CAN_ID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 0;
        case SIM_BOT -> 0;
        default -> 0;
      };

  // Hopper run mode
  // TODO: Set real speeds
  public static enum RunMode {
    /** All hopper motors stopped */
    STOPPED(0, 0),
    /** FEEDER running at low speed to push fuel to the back, LIFTER stopped */
    FUEL_STORE(30, 0),
    /** FEEDER running at high speed to send balls to the LIFTER, LIFTER running */
    FIRING(60, 60),
    /** All hopper motors running in reverse in case of jams */
    REVERSE(-60, -60);

    public int feederVelocityRadPerSec;

    public int lifterVelocityRadPerSec;

    private RunMode(int FEEDERVelocity, int LIFTERVelocity) {
      feederVelocityRadPerSec = FEEDERVelocity;
      lifterVelocityRadPerSec = LIFTERVelocity;
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
  public static final PIDConfig FEEDER_PID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new PIDConfig(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConfig(0.85, 0.0, 0.0);
        default -> new PIDConfig(1.0, 0.0, 0.0);
      };
  public static final PIDConfig LIFTER_PID =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new PIDConfig(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConfig(0.85, 0.0, 0.0);
        default -> new PIDConfig(1.0, 0.0, 0.0);
      };

  // Feedforward constants
  // TODO: Add real values
  public static final FeedForwardConfigRecord FEEDER_FF =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        case SIM_BOT -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        default -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
      };
  public static final FeedForwardConfigRecord LIFTER_FF =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        case SIM_BOT -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
        default -> new FeedForwardConfigRecord(0.0, 0.0, 0.0);
      };

  // Gear ratios
  // TODO: Add real values (thank you design team)
  public static final double FEEDER_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1.0;
        case SIM_BOT -> 1.0;
        default -> 1.0;
      };
  public static final double LIFTER_GEAR_RATIO =
      switch (Constants.getRobot()) {
        case PRESEASON_2026 -> 1.0;
        case SIM_BOT -> 1.0;
        default -> 1.0;
      };
}
