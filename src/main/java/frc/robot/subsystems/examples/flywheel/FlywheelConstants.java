package frc.robot.subsystems.examples.flywheel;

import frc.robot.Constants;
import frc.robot.utility.records.FeedforwardConstants;
import frc.robot.utility.records.PIDConstants;

public class FlywheelConstants {

  // --- Flywheel config ---

  public record FlywheelConfig(int motorID, boolean inverted) {}

  public static final FlywheelConfig FLYWHEEL_CONFIG =
      switch (Constants.getRobot()) {
        case REEFSCAPE_2025 -> new FlywheelConfig(12, false);
        default -> new FlywheelConfig(0, false);
      };

  public static final PIDConstants PID_CONFIG =
      switch (Constants.getRobot()) {
        default -> new PIDConstants(1.0, 0, 0);
      };

  public static final FeedforwardConstants FEEDFORWARD_CONFIG =
      switch (Constants.getRobot()) {
        default -> new FeedforwardConstants(0.1, 0.05, 0);
      };

  // --- Flywheel constants ---
  public static final double GEAR_RATIO = 1.0 / 2.0;
}
