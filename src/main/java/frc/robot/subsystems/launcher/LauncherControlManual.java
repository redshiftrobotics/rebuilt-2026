package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import org.littletonrobotics.junction.AutoLogOutput;

public class LauncherControlManual {

  public enum ManualLaunchMode {
    Y(RadiansPerSecond.of(500.0), Rotation2d.fromDegrees(0.0)),
    X(RadiansPerSecond.of(400.0), Rotation2d.fromDegrees(0.0)),
    A(RadiansPerSecond.of(300.0), Rotation2d.fromDegrees(0.0)),
    B(RadiansPerSecond.of(200.0), Rotation2d.fromDegrees(0.0));

    private final AngularVelocity channelVelocity;
    private AngularVelocity shift;

    private final Rotation2d hoodAngle;
    private Rotation2d hoodAngleShift;

    private ManualLaunchMode(AngularVelocity velocity, Rotation2d hoodAngle) {
      this.channelVelocity = velocity;
      this.hoodAngle = hoodAngle;
      resetShift();
    }

    public void resetShift() {
      this.shift = RadiansPerSecond.zero();
      this.hoodAngleShift = Rotation2d.kZero;
    }

    public void shiftVelocity(AngularVelocity shift) {
      this.shift = this.shift.plus(shift);
    }

    public void shiftHoodAngle(Rotation2d shift) {
      this.hoodAngleShift = this.hoodAngleShift.plus(shift);
    }

    public AngularVelocity getVelocity() {
      return channelVelocity.plus(shift);
    }

    public Rotation2d getHoodAngle() {
      return hoodAngle.plus(hoodAngleShift);
    }

    @Override
    public String toString() {
      return String.format(
          "%s(%.2f r/s, %.2f deg)",
          name(), getVelocity().in(RadiansPerSecond), getHoodAngle().getDegrees());
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

  public LauncherState get() {
    return new LauncherState(
        currentManualLaunchMode.getVelocity(), currentManualLaunchMode.getHoodAngle());
  }

  public void setMode(ManualLaunchMode mode) {
    this.currentManualLaunchMode = mode;
  }

  public void shiftVelocity(AngularVelocity shift) {
    currentManualLaunchMode.shiftVelocity(shift);
  }

  public void resetShift() {
    currentManualLaunchMode.resetShift();
  }
}
