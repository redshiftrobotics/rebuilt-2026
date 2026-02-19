package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConfig;

public class IntakeConstants {

  public class IntakeWheelConstants {
    public static final int CAN_ID = 5;
    public static final double GEAR_RATIO = 1.0 / 3.0;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE_MODE = true;

    public static final double SPEED_INTAKING = 1.0;
  }

  public class SlapdownConstants {
    public static final int CAN_ID = 14;
    public static final double GEAR_RATIO = (1.0 / 4.0) * (1.0 / 4.0) * (1.0 / 2.0);
    public static final boolean INVERTED = false;

    public static final Rotation2d ABSOLUTE_ENCODER_ZERO = Rotation2d.fromRotations(0);

    public static final Rotation2d DOWN_SETPOINT = Rotation2d.fromDegrees(0);
    public static final Rotation2d UP_SETPOINT = Rotation2d.fromDegrees(157);
    public static final Rotation2d INCREMENT_SETPOINT = Rotation2d.fromDegrees(5);

    public static final PIDConfig PID =
        switch (Constants.getRobot()) {
          case REBUILT_2026 -> new PIDConfig(3, 0, 0);
          case SIM_BOT -> new PIDConfig(3, 0, 1);
          default -> new PIDConfig(0, 0, 0);
        };
  }
}
