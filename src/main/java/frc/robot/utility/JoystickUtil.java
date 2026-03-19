package frc.robot.utility;

import edu.wpi.first.math.MathUtil;

public class JoystickUtil {

  public static final double JOYSTICK_DEADBAND = 0.15;

  public static double applySimpleDeadband(double value, double deadband) {
    if (Math.abs(value) < JOYSTICK_DEADBAND) {
      return 0.0;
    }
    return value;
  }

  public static double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
  }

  public static boolean isDeadband(double value) {
    return Math.abs(value) < JOYSTICK_DEADBAND;
  }
}
