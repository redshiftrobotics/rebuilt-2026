package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.examples.flywheel.MotorConstants;

public class HopperConstants {

  public static final MotorConstants FEEDER_CONSTANTS =
      new MotorConstants(17, (1.0 / 3.0) * (1.0 / 3.0), true, false, 30);
  public static final MotorConstants LIFTER_CONSTANTS =
      new MotorConstants(13, (1.0 / 3.0) * (1.0 / 3.0) * (1.0 / 3.0), true, false, 40);

  public static enum HopperRunMode {
    STOPPED(0, 0),
    FUEL_STORE(0.5, 0.0),
    FIRING(1, 1),
    REVERSE(-1, -0.7),
    PREP_SHOT(0, -0.3);

    public double feederDutyCycle;
    public double lifterDutyCycle;

    private HopperRunMode(double feederDutyCycle, double lifterDutyCycle) {
      this.feederDutyCycle = feederDutyCycle;
      this.lifterDutyCycle = lifterDutyCycle;
    }

    @Override
    public String toString() {
      return String.format("%s(feeder=%s, lifter=%s)", name(), feederDutyCycle, lifterDutyCycle);
    }
  }

  public static final DCMotor FEEDER_MOTOR = DCMotor.getNEO(1);
  public static final DCMotor LIFTER_MOTOR = DCMotor.getNEO(1);

  public static final double MAX_FEEDER_SPEED =
      FEEDER_MOTOR.withReduction(1.0 / FEEDER_CONSTANTS.gearRatio()).freeSpeedRadPerSec;
  public static final double MAX_LIFTER_SPEED =
      LIFTER_MOTOR.withReduction(1.0 / LIFTER_CONSTANTS.gearRatio()).freeSpeedRadPerSec;
}
