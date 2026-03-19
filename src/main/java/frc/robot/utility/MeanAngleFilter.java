package frc.robot.utility;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.DoubleCircularBuffer;

/**
 * A class that implements a moving-window mean filter for angles. Useful for reducing measurement
 * noise by averaging angles over a sliding window while properly handling wrap-around.
 */
public class MeanAngleFilter {
  private final DoubleCircularBuffer m_valueBuffer;
  private final int m_size;

  /**
   * Creates a new MeanAngleFilter.
   *
   * @param size The number of samples in the moving window.
   */
  public MeanAngleFilter(int size) {
    // Circular buffer of values currently in the window (stored as radians)
    m_valueBuffer = new DoubleCircularBuffer(size);
    // Size of rolling window
    m_size = size;
  }

  /**
   * Calculates the moving-window mean for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The mean of the moving window, updated to include the next value.
   */
  public Rotation2d calculate(Rotation2d next) {
    // If buffer is at max size, remove oldest value
    if (m_valueBuffer.size() >= m_size) {
      m_valueBuffer.removeLast();
    }

    // Add next value to circular buffer
    m_valueBuffer.addFirst(next.getRadians());

    // Calculate mean using vector averaging
    return calculateMean();
  }

  /**
   * Calculates the mean angle using vector averaging. This properly handles wrap-around by
   * converting angles to unit vectors, averaging them, and converting back.
   */
  private Rotation2d calculateMean() {
    int bufferSize = m_valueBuffer.size();
    if (bufferSize == 0) {
      return Rotation2d.kZero;
    }

    // Sum of x and y components of unit vectors
    double sumX = 0.0;
    double sumY = 0.0;

    for (int i = 0; i < bufferSize; i++) {
      double angle = m_valueBuffer.get(i);
      sumX += Math.cos(angle);
      sumY += Math.sin(angle);
    }

    // Average the components
    double avgX = sumX / bufferSize;
    double avgY = sumY / bufferSize;

    // Convert back to angle using atan2
    return new Rotation2d(avgX, avgY);
  }

  /**
   * Returns the last value added to the MeanAngleFilter.
   *
   * @return The last value.
   */
  public Rotation2d lastValue() {
    return new Rotation2d(m_valueBuffer.getFirst());
  }

  /** Resets the filter, clearing the window of all elements. */
  public void reset() {
    m_valueBuffer.clear();
  }
}
