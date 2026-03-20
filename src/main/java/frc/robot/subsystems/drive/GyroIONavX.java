package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * IO implementation for NavX MXP Gyroscope/IMU
 *
 * <p>https://www.andymark.com/products/navx2-mxp-robotics-navigation-sensor
 */
public class GyroIONavX implements GyroIO {
    private static final NavXComType SERIAL_PORT_ID = NavXComType.kMXP_SPI;

    private final AHRS navX;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    /** Create a new NaxX IMU */
    public GyroIONavX(boolean phoenixOdometry) {
        navX = new AHRS(SERIAL_PORT_ID, (int) DriveConstants.ODOMETRY_FREQUENCY_HERTZ);

        if (phoenixOdometry) {
            yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
        } else {
            yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getYaw);
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();

        // The NavX is clock-wise positive, but the WPILib coordinate system is counter-clockwise
        // positive

        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system
        // https://pdocs.kauailabs.com/navx-mxp/installation/orientation-2/

        inputs.yawPosition = navX.getRotation2d();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
        inputs.rotation3d = navX.getRotation3d();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void zeroGyro() {
        navX.reset();
    }
}
