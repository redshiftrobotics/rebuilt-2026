package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {

  private static SparkOdometryThread instance = null;

  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();

  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();

  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier = new Notifier(this::run);

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("SparkOdometryThread");
  }

  /** Starts the notifier thread. */
  public void start() {
    if (timestampQueues.size() > 0) {
      final double odometryPeriodSeconds = 1.0 / DriveConstants.ODOMETRY_FREQUENCY_HERTZ;
      notifier.startPeriodic(odometryPeriodSeconds);
    }
  }

  /** Registers a Spark signal to be read from the thread. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }

    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Save new data to queues
    Drive.odometryLock.lock();
    try {
      // Get sample timestamp
      double timestampSeconds = RobotController.getFPGATime() / 1e6;

      // Read Spark values, mark invalid in case of error
      double[] sparkValues = new double[sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.get(i).getAsDouble();
        if (!sparks.get(i).getLastError().equals(REVLibError.kOk)) {
          isValid = false;
        }
      }

      // If valid, add values to queues
      if (isValid) {
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueues.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestampSeconds);
        }
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
