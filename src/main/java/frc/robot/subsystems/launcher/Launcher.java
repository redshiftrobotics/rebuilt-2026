package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.launcher.LauncherConstants.ChannelConstants;
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
    MATHEMATICAL,
    INTERPOLATION,
    DASHBOARD_TUNING
  }

  public record LauncherState(double wheelRadPerSec, double hoodPosition) {
    public static LauncherState zero() {
      return new LauncherState(0, 0);
    }
  }

  public final TunablePID flywheelPID =
      new TunablePID(getName() + "/PID", ChannelConstants.FLYWHEEL_PID);
  public final TunableFF flywheelFF = new TunableFF(getName() + "/FF", ChannelConstants.FF);

  public final TunableNumber WHEEL_TOLERANCE_RAD_PER_SEC =
      new TunableNumber(getName() + "/WheelToleranceRadPerSec", 5.0);

  public static TunableNumber HOOD_POSITION_TOLERANCE =
      new TunableNumber("Launcher/HoodPositionToleranceDeg", 2.0);

  public static TunableNumber DEBOUNCE_TIME_AT_GOAL =
      new TunableNumber("Launcher/DebounceTimeAtGoal", 0.75);

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final List<ChannelIO> channelIOs;
  private final List<ChannelIOInputsAutoLogged> channelInputs;
  private final List<Alert> channelDisconnectedAlerts;

  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> robotVelocitySupplier = null;

  private boolean running = false;
  private LauncherRunMode mode = LauncherRunMode.MANUAL;

  private Supplier<LauncherState> manualModeState = () -> LauncherState.zero();

  private LauncherState runningDesiredState = LauncherState.zero();

  private Rotation2d robotYaw = Rotation2d.kZero;

  private final Debouncer atGoalDebouncer =
      new Debouncer(DEBOUNCE_TIME_AT_GOAL.get(), Debouncer.DebounceType.kRising);
  private boolean atGoalDebounced;

  public static Launcher create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Launcher(
            new HoodIOActuator(),
            new ChannelIOTalonFX("Left", ChannelConstants.LEFT_CONFIG),
            new ChannelIOTalonFX("Center", ChannelConstants.CENTER_CONFIG),
            new ChannelIOTalonFX("Right", ChannelConstants.RIGHT_CONFIG));
      case METALBOT_2:
      case SIM_BOT:
        return new Launcher(
            new HoodIOSim(false),
            new ChannelIOSim("Left", ChannelConstants.LEFT_CONFIG),
            new ChannelIOSim("Center", ChannelConstants.CENTER_CONFIG),
            new ChannelIOSim("Right", ChannelConstants.RIGHT_CONFIG));
      default:
        return new Launcher(new HoodIO() {});
    }
  }

  /** Creates a new Launcher. */
  public Launcher(HoodIO hoodIO, ChannelIO... channelIOs) {
    this.hoodIO = hoodIO;
    this.channelIOs = List.of(channelIOs);

    updatePID();
    updateFF();

    channelInputs = new ArrayList<ChannelIOInputsAutoLogged>();
    channelDisconnectedAlerts = new ArrayList<Alert>();
    for (int i = 0; i < channelIOs.length; i++) {
      this.channelIOs.get(i).setPID(ChannelConstants.FLYWHEEL_PID);
      channelInputs.add(new ChannelIOInputsAutoLogged());
      channelDisconnectedAlerts.add(
          new Alert(channelIOs[i].getName() + " channel disconnected", AlertType.kError));
    }

    stop();

    SmartDashboard.putNumber("LauncherTuning/DesiredWheelRadPerSec", 0);
    SmartDashboard.putNumber("LauncherTuning/DesiredHoodPosition", 0);
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

  public LauncherRunMode getMode() {
    return mode;
  }

  public void setDutyCycle(double dutyCycle) {
    for (ChannelIO channel : channelIOs) {
      channel.setDutyCycle(dutyCycle);
    }
  }

  private void setChannelVelocities(double radPerSec) {
    for (ChannelIO channel : channelIOs) {
      channel.setVelocity(radPerSec);
    }
  }

  public void configure(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;
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
    }

    flywheelPID.ifChanged(hashCode(), this::updatePID);
    flywheelFF.ifChanged(hashCode(), this::updateFF);

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds robotRelativeVelocity = robotVelocitySupplier.get();

    switch (mode) {
      case INTERPOLATION:
        {
          LaunchCalculator.LaunchingParameters parameters =
              LaunchCalculator.getInstance().getParameters(robotPose, robotRelativeVelocity);

          setRunningDesiredState(
              new LauncherState(parameters.wheelRadPerSec(), parameters.hoodPosition()));
          robotYaw = parameters.driveAngle();

          SmartDashboard.putNumber("LauncherTuning/CalculatorDistance", parameters.distance());
          SmartDashboard.putNumber(
              "LauncherTuning/CalculatorDistanceAdjustedInches",
              Units.metersToInches(
                  parameters.distance()
                      - FieldConstants.Hub.width / 2
                      - DriveConstants.BUMPER_TO_BUMPER.getX() / 2
                      + LauncherConstants.ROBOT_TO_LAUNCHER.getX()));

          break;
        }
      case MATHEMATICAL:
        {
          MathematicalShotCalculator.MathematicalShotParameters parameters =
              MathematicalShotCalculator.calculateAndSetShot(
                  robotPose, robotRelativeVelocity, !hoodInputs.isAdjustable);

          setRunningDesiredState(
              new LauncherState(parameters.getWheelRadPerSec(), parameters.getHoodPosition()));
          robotYaw = parameters.yaw();
          break;
        }
      case MANUAL:
        {
          // fallback to simple aiming option
          robotYaw =
              LaunchCalculator.getStationaryAimedPose(robotPose.getTranslation()).getRotation();
          setRunningDesiredState(manualModeState.get());
          break;
        }
      case DASHBOARD_TUNING:
        {
          LaunchCalculator.LaunchingParameters parameters =
              LaunchCalculator.getInstance().getParameters(robotPose, robotRelativeVelocity);

          double velocity = SmartDashboard.getNumber("LauncherTuning/DesiredWheelRadPerSec", 0);
          double hood = SmartDashboard.getNumber("LauncherTuning/DesiredHoodPosition", 0);
          setRunningDesiredState(new LauncherState(velocity, hood));

          // NOTE: This distance is launcher to center of target (and is what the calculator uses)
          // When entering distances, use this over measuring by hand if possible (assuming good
          // tags),
          SmartDashboard.putNumber("LauncherTuning/CalculatorDistance", parameters.distance());

          // Alternate option (front of bumper to closest hub face)
          SmartDashboard.putNumber(
              "LauncherTuning/CalculatorDistanceAdjustedInches",
              Units.metersToInches(
                  parameters.distance()
                      - FieldConstants.Hub.width / 2
                      - DriveConstants.BUMPER_TO_BUMPER.getX() / 2
                      + LauncherConstants.ROBOT_TO_LAUNCHER.getX()));

          robotYaw = parameters.driveAngle();
          break;
        }
    }

    boolean atGoal = isReady();
    DEBOUNCE_TIME_AT_GOAL.ifChanged(hashCode(), atGoalDebouncer::setDebounceTime);
    atGoalDebounced = atGoalDebouncer.calculate(atGoal);

    Logger.recordOutput(
        getName() + "/desiredRobotPose", new Pose2d(robotPose.getTranslation(), robotYaw));

    SmartDashboard.putNumber(
        "LauncherTuning/MeasuredWheelRadPerSec", getMeasuredState().wheelRadPerSec());
    SmartDashboard.putNumber(
        "LauncherTuning/DesiredWheelRadPerSec", runningDesiredState.wheelRadPerSec());
    SmartDashboard.putNumber(
        "LauncherTuning/DesiredHoodPosition", runningDesiredState.hoodPosition());

    if (running) {
      hoodIO.setPosition(runningDesiredState.hoodPosition());
      setChannelVelocities(runningDesiredState.wheelRadPerSec());
    } else {
      hoodIO.setPosition(0); // Lower the hood to avoid collisions when not in use
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
    return running ? runningDesiredState : LauncherState.zero();
  }

  @AutoLogOutput(key = "Launcher/measuredState")
  public LauncherState getMeasuredState() {
    return new LauncherState(
        channelInputs.stream().mapToDouble(c -> c.velocityRadPerSec).average().orElse(0.0),
        (hoodInputs.positionLeft + hoodInputs.positionRight) / 2);
  }

  @AutoLogOutput(key = "Launcher/isReady")
  public boolean isReady() {
    LauncherState desired = getDesiredState();

    double desiredVelocity = desired.wheelRadPerSec();
    double velocityTolerance = WHEEL_TOLERANCE_RAD_PER_SEC.get();

    for (ChannelIO.ChannelIOInputs channel : channelInputs) {
      if (!MathUtil.isNear(desiredVelocity, channel.velocityRadPerSec, velocityTolerance)) {
        return false;
      }
    }

    if (!hoodInputs.isAdjustable) {
      double desiredHood = desired.hoodPosition();
      double hoodTolerance = HOOD_POSITION_TOLERANCE.get();

      boolean leftReady = MathUtil.isNear(desiredHood, hoodInputs.positionLeft, hoodTolerance);

      boolean rightReady = MathUtil.isNear(desiredHood, hoodInputs.positionRight, hoodTolerance);

      if (!leftReady || !rightReady) {
        return false;
      }
    }

    return true;
  }

  @AutoLogOutput(key = "Launcher/isReadyDebounced")
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
