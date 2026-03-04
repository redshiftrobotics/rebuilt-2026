package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLogOutput;

public class LauncherControlManual {

  public enum ManualLaunchMode {
    Y(RadiansPerSecond.of(500.0)),
    X(RadiansPerSecond.of(400.0)),
    A(RadiansPerSecond.of(300.0)),
    B(RadiansPerSecond.of(200.0));

    private final AngularVelocity channelVelocity;
    private AngularVelocity shift;

    private ManualLaunchMode(AngularVelocity velocity) {
      this.channelVelocity = velocity;
      resetShift();
    }

    public void resetShift() {
      this.shift = RadiansPerSecond.zero();
    }

    public void shift(AngularVelocity shift) {
      this.shift = this.shift.plus(shift);
    }

    public AngularVelocity get() {
      return channelVelocity.plus(shift);
    }

    @Override
    public String toString() {
      return String.format("%s(%.2f r/s)", name(), get().in(RadiansPerSecond));
    }
  }

  public ManualLaunchMode currentManualLaunchMode;

  public LauncherControlManual(ManualLaunchMode initialMode) {
    this.currentManualLaunchMode = initialMode;
  }

  @AutoLogOutput(key = "Launcher/ManualLauncherControl/manualLaunchMode")
  public String getMode() {
    return currentManualLaunchMode.toString();
  }

  public AngularVelocity get() {
    return currentManualLaunchMode.get();
  }

  public void setMode(ManualLaunchMode mode) {
    this.currentManualLaunchMode = mode;
  }

  public void shiftVelocity(AngularVelocity shift) {
    currentManualLaunchMode.shift(shift);
  }

  public void resetShift() {
    currentManualLaunchMode.resetShift();
  }
}
