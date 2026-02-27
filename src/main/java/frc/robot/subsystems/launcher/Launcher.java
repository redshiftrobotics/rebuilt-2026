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
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import frc.robot.utility.tunable.TunableNumber;
import frc.robot.utility.tunable.TunableNumbers.TunableFF;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {

  public enum LauncherRunMode {
    MANUAL,
    AUTOMATIC,
    INTERPOLATION,
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
  private LauncherRunMode mode;

  private Supplier<AngularVelocity> manualModeDesiredVelocity = () -> RadiansPerSecond.zero();
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

  public void start() {
    running = true;
  }

  public void stop() {
    running = false;
  }

  public void setMode(LauncherRunMode mode) {
    this.mode = mode;
  }

  public void setManualModeSupplier(Supplier<AngularVelocity> setpoint) {
    this.manualModeDesiredVelocity = setpoint;
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

    Translation2d hubTranslation =
        FieldConstants.Hub.topCenterPoint.toTranslation2d().minus(robotPose.getTranslation());

    ShotParameters parameters =
        ShotCalculator.method1(hubTranslation, robotVelocitySupplier.get(), hoodIO.hoodType());

    AngularVelocity runningVelocity = RadiansPerSecond.zero();
    Rotation2d runningHoodPitch = Rotation2d.kZero;

    switch (mode) {
      case MANUAL:
        runningVelocity = manualModeDesiredVelocity.get();
        robotYaw = parameters.yaw();
        break;
      case AUTOMATIC:
        runningHoodPitch = parameters.pitch();
        runningVelocity = calculateWheelVelocity(parameters.velocity());
        robotYaw = parameters.yaw();
        break;
      case INTERPOLATION:
        runningVelocity = RadiansPerSecond.of(InterpolationShotCalculator.calculateWheelVelocity(hubTranslation.getNorm()));
        robotYaw = parameters.yaw();
        break;
    }

    hoodIO.setAngle(runningHoodPitch);

    if (running) {
      setChannelVelocities(runningVelocity);
    } else {
      stopChannelMotors(); // Coast to slow down
    }
  }

  public Rotation2d getRobotYaw() {
    return robotYaw;
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

  private AngularVelocity calculateWheelVelocity(LinearVelocity ballVelocity) {
    ballVelocity =
        ballVelocity
            .times(LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER)
            .times(LauncherConstants.LAUNCHER_TUNING_PARAMETER.get());
    return RadiansPerSecond.of(
        ballVelocity.in(MetersPerSecond) / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters));
  }

  private void updatePID() {
    channelIOs.forEach(m -> m.setPID(flywheelPID.get()));
  }

  private void updateFF() {
    channelIOs.forEach(m -> m.setFF(flywheelFF.get()));
  }
}
