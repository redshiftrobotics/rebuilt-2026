package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumbers.TunableFF;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {

  public enum LauncherRunMode {
    MANUAL,
    INTERPOLATION,
    AUTOMATIC;
  }

  public final TunablePID flywheelPID =
      new TunablePID(getName() + "/PID", LauncherConstants.FLYWHEEL_PID);
  public final TunableFF flywheelFF = new TunableFF(getName() + "/FF", LauncherConstants.FF);

  public final TunableNumber LAUNCHER_VELOCITY_TOLERANCE =
      new TunableNumber(getName() + "/VelocityTolerance", 15.0); // in radians per second

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final List<ChannelIO> channelIOs;
  private final List<ChannelIOInputsAutoLogged> channelInputs;
  private final List<Alert> channelDisconnectedAlerts;
  private final List<Alert> channelConfigFailAlerts;

  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<Translation2d> robotVelocitySupplier = null;

  private boolean running = false;
  private LauncherRunMode mode = LauncherRunMode.MANUAL;

  private Supplier<AngularVelocity> manualModeDesiredVelocity = () -> RadiansPerSecond.zero();
  private Supplier<Rotation2d> manualModeDesiredAngle = () -> LauncherConstants.FIXED_LAUNCH_ANGLE;
  private AngularVelocity desiredVelocity = RadiansPerSecond.zero();

  private Rotation2d robotYaw = Rotation2d.kZero;

  public static Launcher create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Launcher(
            new HoodIOFixed(),
            new ChannelIOTalonFX("Left", LauncherConstants.LEFT_CONSTANTS),
            new ChannelIOTalonFX("Center", LauncherConstants.CENTER_CONSTANTS),
            new ChannelIOTalonFX("Right", LauncherConstants.RIGHT_CONSTANTS));
      case SIM_BOT:
        return new Launcher(
            new HoodIOFixed(),
            new ChannelIOSim("Left", LauncherConstants.LEFT_CONSTANTS),
            new ChannelIOSim("Center", LauncherConstants.CENTER_CONSTANTS),
            new ChannelIOSim("Right", LauncherConstants.RIGHT_CONSTANTS));
      default:
        return new Launcher(new HoodIO() {});
    }
  }

  /** Creates a new Template. */
  public Launcher(HoodIO hoodIO, ChannelIO... channelIOs) {
    this.hoodIO = hoodIO;
    this.channelIOs = List.of(channelIOs);

    updatePID();
    updateFF();

    channelInputs = new ArrayList<ChannelIOInputsAutoLogged>();
    channelDisconnectedAlerts = new ArrayList<Alert>();
    channelConfigFailAlerts = new ArrayList<Alert>();
    for (int i = 0; i < channelIOs.length; i++) {
      this.channelIOs.get(i).setPID(LauncherConstants.FLYWHEEL_PID);
      channelInputs.add(new ChannelIOInputsAutoLogged());
      channelDisconnectedAlerts.add(
          new Alert(channelIOs[i].getName() + " channel disconnected", AlertType.kError));
      channelConfigFailAlerts.add(
          new Alert(channelIOs[i].getName() + " channel config failed", AlertType.kError));
    }

    stop();
  }

  public void setManualModeVelocitySupplier(Supplier<AngularVelocity> setpoint) {
    this.manualModeDesiredVelocity = setpoint;
  }

  public void setManualModeAngleSupplier(Supplier<Rotation2d> setpoint) {
    this.manualModeDesiredAngle = setpoint;
  }

  public void setDutyCycle(double dutyCycle) {
    desiredVelocity = RadiansPerSecond.zero();
    for (ChannelIO channel : channelIOs) {
      channel.setDutyCycle(dutyCycle);
    }
  }

  private void stopChannelMotors() {
    desiredVelocity = RadiansPerSecond.of(0.0);
    for (ChannelIO channel : channelIOs) {
      channel.stop();
    }
    Logger.recordOutput(getName() + "/desiredVelocityRadPerSec", 0);
  }

  public void stop() {
    running = false;
    stopChannelMotors();
  }

  public void start() {
    running = true;
  }

  public void start(LauncherRunMode mode) {
    this.mode = mode;
    running = true;
  }

  public void setRunning(boolean running) {
    this.running = running;
  }

  public void setMode(LauncherRunMode mode) {
    this.mode = mode;
  }

  private void setChannelVelocities(AngularVelocity wheelVelocity) {
    this.desiredVelocity = wheelVelocity;
    for (ChannelIO channel : channelIOs) {
      channel.setVelocity(wheelVelocity);
    }
  }

  public void configure(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> robotVelocitySupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs(getName() + "/Hood", hoodInputs);

    for (int i = 0; i < channelIOs.size(); i++) {
      channelIOs.get(i).updateInputs(channelInputs.get(i));
      Logger.processInputs(
          getName() + "/Channel" + channelIOs.get(i).getName(), channelInputs.get(i));
      channelDisconnectedAlerts.get(i).set(!channelInputs.get(i).motorConnected);
      channelConfigFailAlerts.get(i).set(channelInputs.get(i).pushedConfigFault);
    }

    flywheelPID.ifChanged(hashCode(), this::updatePID);
    flywheelFF.ifChanged(hashCode(), this::updateFF);

    Pose2d robotPose = robotPoseSupplier.get();

    Translation2d hubLocation =
        FieldConstants.Hub.topCenterPoint.toTranslation2d().minus(robotPose.getTranslation());

    Function<Double, Rotation2d> pitchCalculator = getPitchCalculator();
    Translation2d hubTranslation = ShotCalculator.adjustedHubPosition(hubLocation, robotVelocitySupplier.get(), pitchCalculator);

    AngularVelocity runningVelocity = RadiansPerSecond.zero();
    Rotation2d runningHoodPitch = Rotation2d.kZero;
    robotYaw = hubTranslation.getAngle();

    double distance = hubTranslation.getNorm();
    runningHoodPitch = pitchCalculator.apply(distance);
    runningVelocity = calculateVelocity(hubTranslation.getNorm());

    hoodIO.setAngle(runningHoodPitch);

    if (running) {
      setChannelVelocities(runningVelocity);
    } else {
      stopChannelMotors(); // Coast to slow down
    }
  }

  @AutoLogOutput(key = "Launcher/isRunning")
  public boolean isRunning() {
    return running;
  }

  @AutoLogOutput(key = "Launcher/desiredVelocity")
  public AngularVelocity getDesiredVelocity() {
    return desiredVelocity;
  }

  @AutoLogOutput(key = "Launcher/averageVelocity")
  public double getAverageVelocity() {
    return channelInputs.stream().mapToDouble(c -> c.velocityRadPerSec).average().orElse(0.0);
  }

  @AutoLogOutput(key = "Launcher/runMode")
  public LauncherRunMode getRunMode() {
    return mode;
  }

  @AutoLogOutput(key = "Launcher/isReady")
  public boolean isReady() {
    boolean ready = true;
    for (ChannelIO.ChannelIOInputs channelIO : channelInputs) {
      boolean channelReady =
          MathUtil.isNear(
              desiredVelocity.in(RadiansPerSecond),
              channelIO.velocityRadPerSec,
              LAUNCHER_VELOCITY_TOLERANCE.get());
      ready = ready && channelReady;
    }
    return ready;
  }

  private void updatePID() {
    channelIOs.forEach(m -> m.setPID(flywheelPID.get()));
  }

  private void updateFF() {
    channelIOs.forEach(m -> m.setFF(flywheelFF.get()));
  }

  public Rotation2d getRobotYaw() {
    return robotYaw;
  }
  /**
   * @param double the distance in meters from the hub
   * @return the pitch of the launch trajectory
   */
  private Function<Double, Rotation2d> getPitchCalculator() {
    if (hoodIO.hoodType() == HoodType.FIXED) {
      return (dist) -> LauncherConstants.FIXED_LAUNCH_ANGLE;
    }
    switch (mode) {
      case MANUAL:
        return (Double dist) -> manualModeDesiredAngle.get();
      case INTERPOLATION:
        return LauncherControlInterpolation::calculateHoodPitch;
      case AUTOMATIC:
        return (Double dist) -> LauncherControlAutomatic.calculatePitch(Meters.of(dist));
    }
    return (Double dist) -> Rotation2d.kZero;
  }
  private AngularVelocity calculateVelocity(double distance) {
    switch (mode) {
      case MANUAL:
        return manualModeDesiredVelocity.get();
      case INTERPOLATION:
        switch (hoodIO.hoodType()) {
          case FIXED:
            return RadiansPerSecond.of(LauncherControlInterpolation.calculateVelocity(distance));
          case ACTUATOR:
            return RadiansPerSecond.of(LauncherControlInterpolation.calculateVelocityAdjustableHood(distance));
        }
      case AUTOMATIC:
        Rotation2d pitch = LauncherControlAutomatic.calculatePitch(Meters.of(distance));
        LinearVelocity velocity = ShotCalculator.calculateVelocity(distance, pitch);
        velocity = velocity
            .times(LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER)
            .times(LauncherConstants.LAUNCHER_TUNING_PARAMETER.get());
        return RadiansPerSecond.of(
            velocity.in(MetersPerSecond) / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters));
      default:
        return RadiansPerSecond.zero();
    }
  }
}
