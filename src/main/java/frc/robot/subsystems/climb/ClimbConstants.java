package frc.robot.subsystems.climb;

import frc.robot.Constants;

/** Constants for the Climb subsystem. */
public class ClimbConstants {
  public static final int CAN_ID;

  public static final boolean MOTOR_INVERTED;

  public static final double GEAR_RATIO;

  static {
    switch (Constants.getRobot()) {
      default:
        CAN_ID = 0;
        MOTOR_INVERTED = false;
        GEAR_RATIO = 1;
        break;
    }
  }
}
