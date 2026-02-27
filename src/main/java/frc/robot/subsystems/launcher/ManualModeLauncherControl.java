package frc.robot.subsystems.launcher;

public class ManualModeLauncherControl {

  public enum ManualLaunchMode {
    Y(10.0),
    X(20.0),
    A(30.0),
    B(40.0);

    private final double channelVelocityRadPerSec;
    private double shift;

    private ManualLaunchMode(double channelVelocityRadPerSec) {
      this.channelVelocityRadPerSec = channelVelocityRadPerSec;
      resetShift();
    }

    public void resetShift() {
      this.shift = 0.0;
    }

    public void shiftVelocity(double shift) {
      this.shift += shift;
    }

    public double getVelocityRadPerSecond() {
      return channelVelocityRadPerSec + shift;
    }

    @Override
    public String toString() {
      return String.format("%s(velocity=%.2f)", name(), getVelocityRadPerSecond());
    }
  }

  public ManualLaunchMode currentManualLaunchMode;

  public ManualModeLauncherControl(ManualLaunchMode initialMode) {
    this.currentManualLaunchMode = initialMode;
  }

  public ManualLaunchMode getMode() {
    return currentManualLaunchMode;
  }

  public void setMode(ManualLaunchMode mode) {
    this.currentManualLaunchMode = mode;
  }

  public void shiftVelocity(double shift) {
    currentManualLaunchMode.shiftVelocity(shift);
  }

  public void resetShift() {
    currentManualLaunchMode.resetShift();
  }
}
