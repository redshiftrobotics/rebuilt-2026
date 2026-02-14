package frc.robot.subsystems.climb;

import frc.robot.Constants;

/** Constants for the Climb subsystem. */
public class ClimbConstants {
  /** The CAN ID of the limit switch. */
  public static final int SWITCH_ID;
  /** The CAN ID of the motor. */
  public static final int MOTOR_ID;
  /** If the motor should be run inverted (backwards). */
  public static final boolean MOTOR_INVERTED;
  /** The ratio of the gearbox on the motor. */
  public static final double GEAR_RATIO;

  static {
    switch (Constants.getRobot()) {
      default:
        SWITCH_ID = 0;
        MOTOR_ID = 0;
        MOTOR_INVERTED = false;
        GEAR_RATIO = 1;
        break;
    }
  }
}
