package frc.robot.utility;

public class ThresholdController {
  private double activationThreshold;
  private double disableThreshold;

  private boolean isOn;

  private Direction direction;

  public static enum Direction {
    UP,
    DOWN
  }

  /** Creates a new ThresholdController with default thresholds of 0 */
  public ThresholdController() {
    this(0, 0, Direction.UP);
  }

  /**
   * Creates a new ThresholdController with specified thresholds.
   *
   * @param activationThreshold The threshold at which the controller turns ON.
   * @param disableThreshold The threshold at which the controller turns OFF.
   */
  public ThresholdController(double activationThreshold, double disableThreshold) {
    setThresholds(
        activationThreshold,
        disableThreshold,
        activationThreshold < disableThreshold ? Direction.UP : Direction.DOWN);
    isOn = false;
  }

  /**
   * Creates a new ThresholdController with specified thresholds.
   *
   * @param activationThreshold The threshold at which the controller turns ON.
   * @param disableThreshold The threshold at which the controller turns OFF.
   * @param direction The direction of the controller.
   */
  public ThresholdController(
      double activationThreshold, double disableThreshold, Direction direction) {
    setThresholds(activationThreshold, disableThreshold, direction);
    isOn = false;
  }

  /**
   * Sets both the lower and upper thresholds.
   *
   * @param activationThreshold The lower threshold at which the controller turns ON.
   * @param disableThreshold The upper threshold at which the controller turns OFF.
   */
  public void setThresholds(
      double activationThreshold, double disableThreshold, Direction direction) {
    this.direction = direction;
    if (direction == Direction.UP && activationThreshold > disableThreshold) {
      throw new IllegalArgumentException(
          "Activation threshold must be less than the disable threshold in UP direction.");
    } else if (direction == Direction.DOWN && activationThreshold < disableThreshold) {
      throw new IllegalArgumentException(
          "Activation threshold must be greater than the disable threshold in DOWN direction.");
    }
    this.activationThreshold = activationThreshold;
    this.disableThreshold = disableThreshold;
  }

  /**
   * Sets the lower threshold.
   *
   * @param lowerThreshold The lower threshold at which the controller turns ON.
   */
  public void setActivationThreshold(double lowerThreshold) {
    setThresholds(lowerThreshold, disableThreshold, direction);
  }

  /**
   * Sets the upper threshold.
   *
   * @param upperThreshold The upper threshold at which the controller turns OFF.
   */
  public void setDisableThreshold(double upperThreshold) {
    setThresholds(activationThreshold, upperThreshold, direction);
  }

  /**
   * Returns the lower threshold.
   *
   * @return The lower threshold.
   */
  public double getActivationThreshold() {
    return activationThreshold;
  }

  /**
   * Returns the upper threshold.
   *
   * @return The upper threshold.
   */
  public double getDisableThreshold() {
    return disableThreshold;
  }

  /**
   * Returns the direction of the controller. UP means the controller turns ON when the measurement
   * is below the lower threshold and turns OFF when the measurement is above the upper threshold.
   * DOWN means the controller turns ON when the measurement is above the lower threshold and turns
   * OFF when the measurement is below the upper threshold.
   *
   * @return The direction of the controller.
   */
  public Direction getDirection() {
    return direction;
  }

  /**
   * Returns whether the controller is currently ON.
   *
   * @return True if the controller is ON; otherwise, false.
   */
  public boolean isOn() {
    return isOn;
  }

  /**
   * Returns the control output based on the thresholds.
   *
   * @param measurement The current measurement of the process variable.
   * @return 1 if the measurement is below the lower threshold or remains ON until the upper
   *     threshold is reached; otherwise, 0.
   */
  public boolean calculate(double measurement) {
    if (direction == Direction.UP) {
      if (measurement < activationThreshold) {
        isOn = true;
      } else if (measurement > disableThreshold) {
        isOn = false;
      }
    } else {
      if (measurement > activationThreshold) {
        isOn = true;
      } else if (measurement < disableThreshold) {
        isOn = false;
      }
    }
    return isOn;
  }
}
