package frc.robot.subsystems.examples.flywheel;

import frc.robot.Constants;

public class FlywheelConstants {

  // --- Flywheel config ---

  public record FlywheelConfig(int motorID, boolean inverted) {}

  public static final FlywheelConfig FLYWHEEL_CONFIG =
      switch (Constants.getRobot()) {
        case REEFSCAPE_2025 -> new FlywheelConfig(12, false);
        default -> new FlywheelConfig(0, false);
      };

  public record PID(double Kp, double Ki, double Kd) {}

  public static final PID PID_CONFIG =
      switch (Constants.getRobot()) {
        default -> new PID(1.0, 0, 0);
      };

  public record FeedForward(double Ks, double Kv, double Ka) {}

  public static final FeedForward FEED_FORWARD_CONFIG =
      switch (Constants.getRobot()) {
        default -> new FeedForward(0.1, 0.05, 0);
      };

  // --- Flywheel constants ---
  public static final double GEAR_RATIO = 1.0 / 2.0;
}
