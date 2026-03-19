package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConfig;

/**
 * Constants for the Intake subsystem.
 *
 * <p>Contains configuration values for both the intake wheel motor and the slapdown mechanism,
 * including CAN IDs, gear ratios, PID configurations, and setpoint positions.
 */
public class IntakeConstants {

    /** Constants for the intake wheel motor. */
    public class IntakeWheelConstants {
        public static final int CAN_ID = 13;
        public static final double GEAR_RATIO = 1.0 / 3.0;
        public static final boolean INVERTED = true;
        public static final boolean BRAKE_MODE = false;

        public static final DCMotor MOTOR = DCMotor.getNEO(1).withReduction(1.0 / GEAR_RATIO);
    }

    /** Constants for the intake slapdown mechanism. */
    public class SlapdownConstants {
        public static final int CAN_ID = 17;
        public static final double GEAR_RATIO = (1.0 / 4.0) * (1.0 / 4.0) * (1.0 / 2.0);
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_TYPE = DCMotor.getNEO(1).withReduction(1.0 / GEAR_RATIO);

        public static final Rotation2d ABSOLUTE_ENCODER_ZERO = Rotation2d.fromRadians(3.414031536391377);
        public static final boolean ABSOLUTE_ENCODER_INVERTED = true;

        public static final PIDConfig PID =
                switch (Constants.getRobot()) {
                    case REBUILT_2026 -> new PIDConfig(3, 0, 0);
                    case SIM_BOT -> new PIDConfig(3, 0, 1);
                    default -> new PIDConfig(0, 0, 0);
                };
    }
}
