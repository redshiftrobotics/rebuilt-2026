package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.launcher.MathematicalShotCalculator.ShotParameters;
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
    MATHEMATICAL,
    INTERPOLATION;
  }

  public record LauncherState(AngularVelocity flywheelVelocity, Rotation2d hoodAngle) {
    public static LauncherState zero() {
      return new LauncherState(RadiansPerSecond.zero(), Rotation2d.kZero);
    }
  }

  public final TunablePID flywheelPID =
      new TunablePID(getName() + "/PID", LauncherConstants.FLYWHEEL_PID);
  public final TunableFF flywheelFF = new TunableFF(getName() + "/FF", LauncherConstants.FF);

  public final TunableNumber LAUNCHER_VELOCITY_TOLERANCE_RAD_PER_SEC =
      new TunableNumber(getName() + "/VelocityToleranceRadPerSec", 15.0);

  public static TunableNumber HOOD_ANGLE_TOLERANCE_DEG =
      new TunableNumber("Launcher/HoodAngleToleranceDeg", 2.0);

  public static TunableNumber DEBOUNCE_TIME_AT_GOAL =
      new TunableNumber("Launcher/DebounceTimeAtGoal", 0.5);

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final List<ChannelIO> channelIOs;
  private final List<ChannelIOInputsAutoLogged> channelInputs;
  private final List<Alert> channelDisconnectedAlerts;
  private final List<Alert> channelConfigFailAlerts;

  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> robotVelocitySupplier = null;

  private boolean running = false;
  private LauncherRunMode mode = LauncherRunMode.MANUAL;

  private Supplier<LauncherState> manualModeState = () -> LauncherState.zero();

  private LauncherState runningDesiredState = LauncherState.zero();
  private LauncherState desiredState = LauncherState.zero();
  private LauncherState measuredState = LauncherState.zero();

  private Rotation2d robotYaw = Rotation2d.kZero;

  private final Debouncer atGoalDebouncer =
      new Debouncer(DEBOUNCE_TIME_AT_GOAL.get(), Debouncer.DebounceType.kRising);
  private boolean atGoalDebounced;

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

  public void setManualModeState(Supplier<LauncherState> manualModeState) {
    this.manualModeState = manualModeState;
  }

  private void stopChannelMotors() {
    for (ChannelIO channel : channelIOs) {
      channel.stop();
    }
  }

  public void stop() {
    running = false;
  }

  public void start() {
    running = true;
  }

  public void setRunning(boolean running) {
    this.running = running;
  }

  public void setMode(LauncherRunMode mode) {
    this.mode = mode;
  }

  public void setDutyCycle(double dutyCycle) {
    for (ChannelIO channel : channelIOs) {
      channel.setDutyCycle(dutyCycle);
    }
  }

  private void setChannelVelocities(AngularVelocity wheelVelocity) {
    for (ChannelIO channel : channelIOs) {
      channel.setVelocity(wheelVelocity);
    }
  }

  public void configure(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;
    ;
  }

  public void setRunningDesiredState(LauncherState runningDesiredState) {
    this.runningDesiredState = runningDesiredState;
    Logger.recordOutput(getName() + "/runningDesiredState", runningDesiredState);
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

    DEBOUNCE_TIME_AT_GOAL.ifChanged(hashCode(), atGoalDebouncer::setDebounceTime);
    atGoalDebounced = atGoalDebouncer.calculate(isReady());

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds robotRelativeVelocity = robotVelocitySupplier.get();

    if (mode == LauncherRunMode.INTERPOLATION) {
      LaunchingParameters parameters =
          LaunchCalculator.getInstance().getParameters(robotPose, robotRelativeVelocity);

      setRunningDesiredState(new LauncherState(parameters.flywheelSpeed(), parameters.hoodAngle()));
      robotYaw = parameters.driveAngle();

    } else if (mode == LauncherRunMode.MATHEMATICAL) {

      Function<Double, Rotation2d> pitchCalculator =
          switch (hoodIO.hoodType()) {
            case FIXED -> d -> hoodIO.getAngle();
            case ACTUATOR -> d -> MathematicalShotCalculator.calculatePitch(d);
          };

      Translation2d hubLocation =
          FieldConstants.Hub.topCenterPoint.toTranslation2d().minus(robotPose.getTranslation());

      ChassisSpeeds fieldRelativeSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotPose.getRotation());

      Translation2d hubTranslation =
          MathematicalShotCalculator.adjustedHubPosition(
              hubLocation,
              new Translation2d(
                  fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond),
              pitchCalculator);
      Logger.recordOutput(
          getName() + "/HubAdjustment", hubTranslation.minus(hubLocation).getNorm());

      double distance = hubTranslation.getNorm();
      Rotation2d runningHoodPitch = pitchCalculator.apply(distance);

      ShotParameters parameters =
          new ShotParameters(
              MathematicalShotCalculator.calculateVelocity(hubLocation.getNorm(), runningHoodPitch),
              runningHoodPitch);

      double timeOfFlight = MathematicalShotCalculator.timeOfFlight(parameters, hubLocation);
      Logger.recordOutput(getName() + "/timeOfFlight", Math.round(timeOfFlight * 10.0) / 10.0);

      LinearVelocity velocity =
          MathematicalShotCalculator.calculateVelocity(distance, runningHoodPitch);
      velocity = velocity.times(LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER.get());

      AngularVelocity runningVelocity =
          RadiansPerSecond.of(
              velocity.in(MetersPerSecond) / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters));

      measuredState =
          new LauncherState(
              RadiansPerSecond.of(
                  channelInputs.stream()
                      .mapToDouble(c -> c.velocityRadPerSec)
                      .average()
                      .orElse(0.0)),
              hoodIO.getAngle());

      setRunningDesiredState(new LauncherState(runningVelocity, runningHoodPitch));
      this.robotYaw = hubTranslation.getAngle();
    } else if (mode == LauncherRunMode.MANUAL) {
      setRunningDesiredState(manualModeState.get());
    }

    Logger.recordOutput(
        getName() + "/desiredRobotPose", new Pose2d(robotPose.getTranslation(), robotYaw));

    desiredState = running ? runningDesiredState : LauncherState.zero();

    if (running) {
      hoodIO.setAngle(runningDesiredState.hoodAngle());
      setChannelVelocities(runningDesiredState.flywheelVelocity());
    } else {
      hoodIO.setAngle(Rotation2d.kZero); // Lower the hood to avoid collisions when not in use
      stopChannelMotors(); // Coast to slow down
    }
  }

  @AutoLogOutput(key = "Launcher/isRunning")
  public boolean isRunning() {
    return running;
  }

  @AutoLogOutput(key = "Launcher/runMode")
  public LauncherRunMode getRunMode() {
    return mode;
  }

  @AutoLogOutput(key = "Launcher/desiredState")
  public LauncherState getDesiredState() {
    return desiredState;
  }

  @AutoLogOutput(key = "Launcher/measuredState")
  public LauncherState getMeasuredState() {
    return measuredState;
  }

  @AutoLogOutput(key = "Launcher/isReady")
  public boolean isReady() {
    for (ChannelIO.ChannelIOInputs channelIO : channelInputs) {
      if (MathUtil.isNear(
          desiredState.flywheelVelocity().in(RadiansPerSecond),
          channelIO.velocityRadPerSec,
          LAUNCHER_VELOCITY_TOLERANCE_RAD_PER_SEC.get())) {
        return false;
      }
    }
    if (!MathUtil.isNear(
        desiredState.hoodAngle().getDegrees(),
        hoodIO.getAngle().getDegrees(),
        HOOD_ANGLE_TOLERANCE_DEG.get())) {
      return false;
    }
    return true;
  }

  public boolean isReadyDebounced() {
    return atGoalDebounced;
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
}
