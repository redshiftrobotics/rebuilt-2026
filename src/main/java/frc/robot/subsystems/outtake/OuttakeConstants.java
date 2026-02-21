package frc.robot.subsystems.outtake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.common.velocityMotor.MotorConstants;
import frc.robot.utility.records.PIDConfig;

public class OuttakeConstants {
  public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

  public static final double STALL_CURRENT = 120;

  public static final MotorConstants LEFT_CONSTANTS =
      new MotorConstants(3, 1, false, false, STALL_CURRENT);
  public static final MotorConstants MIDDLE_CONSTANTS =
      new MotorConstants(4, 1, false, false, STALL_CURRENT);
  public static final MotorConstants RIGHT_CONSTANTS =
      new MotorConstants(15, 1, false, true, STALL_CURRENT);

  public static final PIDConfig PID =
      switch (Constants.getRobot()) {
        case REBUILT_2026 -> new PIDConfig(1.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConfig(1.0, 0.0, 0.0);
        default -> new PIDConfig(0.0, 0.0, 0.0);
      };
}
