package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConfig;

public class HopperConstants {

  public record MotorConstants(
      int deviceId, double gearRatio, boolean inverted, boolean brakeMode, double stallCurrent) {}

  public static final MotorConstants FEEDER_CONSTANTS =
      new MotorConstants(17, (1.0 / 3.0) * (1.0 / 3.0), true, false, 30);
  public static final MotorConstants LIFTER_CONSTANTS =
      new MotorConstants(13, (1.0 / 3.0) * (1.0 / 3.0) * (1.0 / 3.0), true, false, 40);

  public static final PIDConfig LIFTER_FEEDBACK =
      switch (Constants.getRobot()) {
        case SIM_BOT -> new PIDConfig(1, 0, 0);
        case REBUILT_2026 -> new PIDConfig(1, 0, 0);
        default -> new PIDConfig(0, 0, 0);
      };

  public static enum HopperRunMode {
    STOPPED(0, 0),
    IDLE(0.5, 0.0),
    PREP_SHOT(0, -0.3),
    FIRING(1, 1),
    REVERSE(-1, -0.7);

    public double feederDutyCycle;
    public double lifterVelocityRadPerSec;

    private HopperRunMode(double feederDutyCycle, double lifterVelocityRadPerSec) {
      this.feederDutyCycle = feederDutyCycle;
      this.lifterVelocityRadPerSec = lifterVelocityRadPerSec;
    }

    @Override
    public String toString() {
      return String.format(
          "%s (feeder = %s, lifter = %s)", name(), feederDutyCycle, lifterVelocityRadPerSec);
    }
  }

  public static final DCMotor FEEDER_MOTOR = DCMotor.getNEO(1);
  public static final DCMotor LIFTER_MOTOR = DCMotor.getNEO(1);

  public static final double MAX_FEEDER_SPEED =
      FEEDER_MOTOR.withReduction(1.0 / FEEDER_CONSTANTS.gearRatio()).freeSpeedRadPerSec;
  public static final double MAX_LIFTER_SPEED =
      LIFTER_MOTOR.withReduction(1.0 / LIFTER_CONSTANTS.gearRatio()).freeSpeedRadPerSec;
}
