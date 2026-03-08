package frc.robot.subsystems.launcher;

import frc.robot.subsystems.launcher.Launcher.LauncherState;
import java.util.EnumMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class LauncherControlManual implements Supplier<LauncherState> {

  public enum ManualLaunchMode {
    Y(500.0, 0.0),
    X(400.0, 0.0),
    A(300.0, 0.0),
    B(200.0, 0.0);

    final LauncherState base;

    ManualLaunchMode(double velocity, double hood) {
      this.base = new LauncherState(velocity, hood);
    }
  }

  private static class AdjustableSetpoint {
    final LauncherState base;

    double velocityShift = 0;
    double hoodShift = 0;

    AdjustableSetpoint(LauncherState base) {
      this.base = base;
    }

    LauncherState get() {
      return new LauncherState(
          base.wheelRadPerSec() + velocityShift, base.hoodPosition() + hoodShift);
    }

    void incrementVelocity(double delta) {
      velocityShift += delta;
    }

    void incrementHood(double delta) {
      hoodShift += delta;
    }

    void setHood(double position) {
      hoodShift = position - base.hoodPosition();
    }

    void reset() {
      velocityShift = 0;
      hoodShift = 0;
    }
  }

  private final EnumMap<ManualLaunchMode, AdjustableSetpoint> setpoints =
      new EnumMap<>(ManualLaunchMode.class);

  private ManualLaunchMode mode;

  public LauncherControlManual(ManualLaunchMode initialMode) {
    this.mode = initialMode;

    for (ManualLaunchMode m : ManualLaunchMode.values()) {
      setpoints.put(m, new AdjustableSetpoint(m.base));
    }
  }

  @Override
  public LauncherState get() {
    return setpoints.get(mode).get();
  }

  @AutoLogOutput(key = "Launcher/ManualLauncherControl/manualMode")
  public String getMode() {
    return mode.name();
  }

  public void setMode(ManualLaunchMode mode) {
    this.mode = mode;
  }

  public void incrementVelocity(double delta) {
    setpoints.get(mode).incrementVelocity(delta);
  }

  public void incrementHood(double delta) {
    setpoints.get(mode).incrementHood(delta);
  }

  public void setHood(double position) {
    setpoints.get(mode).setHood(position);
  }

  public void reset() {
    setpoints.get(mode).reset();
  }
}
