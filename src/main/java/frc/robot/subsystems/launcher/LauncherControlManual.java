package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import java.util.EnumMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class LauncherControlManual implements Supplier<LauncherState> {

  public enum ManualLaunchMode {
    Y(DCMotor.getKrakenX60(1).freeSpeedRadPerSec, 1),
    X(450.0, 0.3),
    A(425.0, 0.2),
    B(400.0, 0.0);

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

  public Command setModeCommand(ManualLaunchMode mode) {
    return Commands.runOnce(() -> this.mode = mode)
        .ignoringDisable(true)
        .withName(String.format("Set Manual Launch Mode %s", mode.name()));
  }

  public Command incrementVelocityCommand(double delta) {
    return Commands.runOnce(() -> setpoints.get(mode).incrementVelocity(delta))
        .ignoringDisable(true)
        .withName(String.format("Increment Launch Mode %+f", delta));
  }

  public Command incrementHoodCommand(double delta) {
    return Commands.runOnce(() -> setpoints.get(mode).incrementHood(delta))
        .ignoringDisable(true)
        .withName(String.format("Increment Hood Position %+f", delta));
  }

  public Command resetCommand() {
    return Commands.runOnce(() -> setpoints.get(mode).reset())
        .ignoringDisable(true)
        .withName("Reset Manual Launch Adjustments");
  }
}
