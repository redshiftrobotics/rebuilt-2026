package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/**
 * IO implementation for Pigeon2 IMU
 *
 * <p>https://store.ctr-electronics.com/pigeon-2/
 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Create a new Pigeon2 IMU */
  public GyroIOPigeon2(int deviceID, boolean phoenixOdometry) {
    pigeon = new Pigeon2(deviceID);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    // Initialize the Pigeon2 to the default configuration
    Pigeon2Configuration config = new Pigeon2Configuration();
    pigeon.getConfigurator().apply(config);

    // Tell the Pigeon2 to start with a yaw of 0
    pigeon.setYaw(0);

    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY_HERTZ);

    // Just have it be 50Hz, as its not needed for odometry and is not on a SparkOdometryThread
    yawVelocity.setUpdateFrequency(50);

    pigeon.optimizeBusUtilization();

    if (phoenixOdometry) {
      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    } else {
      yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.rotation3d = pigeon.getRotation3d();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void zeroGyro() {
    pigeon.reset();
  }
}
