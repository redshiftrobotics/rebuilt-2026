package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.utility.AllianceMirrorUtil;
import frc.robot.utility.tunable.TunableNumbers.TunableFF;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class Launcher extends SubsystemBase {

  public final TunablePID flywheelPID = new TunablePID(getName() + "/PID", OuttakeConstants.PID);
  public final TunableFF flywheelFF = new TunableFF(getName() + "/FF", OuttakeConstants.FF);

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final List<ChannelIO> channelIOs;
  private final List<ChannelIOInputsAutoLogged> channelInputs;
  private final List<Alert> channelDisconnectedAlerts;
  private final List<Alert> channelConfigFailAlerts;

  private Supplier<Pose2d> robotPose = null;
  private Supplier<Translation2d> robotVelocity = null;

  private boolean running = false;
  private Rotation2d robotYaw = AllianceMirrorUtil.apply(Rotation2d.kZero);

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

    SmartDashboard.putData(
        "Outtake State",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("Run", () -> running, null);
            for (int i = 0; i < channelIOs.length; i++) {
              final var inputs = channelInputs.get(i);
              builder.addDoubleProperty(
                  channelIOs[i].getName() + " Velocity (rad/sec)",
                  () -> inputs.velocityRadPerSec,
                  null);
              builder.addDoubleProperty(
                  channelIOs[i].getName() + " duty cycle", () -> inputs.appliedDutycycle, null);
            }
          }
        });
  }

  private void stopMotors() {
    for (ChannelIO channel : channelIOs) {
      channel.stop();
    }
  }

  public void stop() {
    this.running = false;
    stopMotors();
  }

  public void start() {
    this.running = true;
  }

  public void setRunning(boolean enabled) {
    this.running = enabled;
    if (enabled == false) stopMotors();
  }

  private void updatePID() {
    channelIOs.forEach(m -> m.setPID(flywheelPID.get()));
  }

  private void updateFF() {
    channelIOs.forEach(m -> m.setFF(flywheelFF.get()));
  }

  public void configure(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> robotVelocitySupplier) {
    robotPose = robotPoseSupplier;
    robotVelocity = robotVelocitySupplier;
  }

  @Override
  public void periodic() {
    flywheelPID.ifChanged(hashCode(), () -> updatePID());
    flywheelFF.ifChanged(hashCode(), () -> updateFF());

    if (running) {
      Translation2d hubTranslation =
          FieldConstants.Hub.topCenterPoint
              .toTranslation2d()
              .minus(robotPose.get().getTranslation());
      ShotParameters parameters =
          ShotCalculator.method1(hubTranslation, robotVelocity.get(), hoodIO.hoodType());

      hoodIO.setAngle(parameters.pitch());
      for (ChannelIO channel : channelIOs) {
        channel.setVelocity(
            RadiansPerSecond.of(
                parameters.velocity().in(MetersPerSecond)
                    * LauncherConstants.LAUNCHER_VELOCITY_MULTIPLIER
                    * LauncherConstants.LAUNCHER_TUNING_PARAMETER.get()
                    / LauncherConstants.LAUNCHER_WHEEL_RADIUS.in(Meters)));
      }
      robotYaw = parameters.yaw();
    }

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs(getName() + "/Hood", hoodInputs);

    for (int i = 0; i < channelIOs.size(); i++) {
      channelIOs.get(i).updateInputs(channelInputs.get(i));
      Logger.processInputs(
          getName() + "/Channel" + channelIOs.get(i).getName(), channelInputs.get(i));
      channelDisconnectedAlerts.get(i).set(channelInputs.get(i).motorConnected);
      channelConfigFailAlerts.get(i).set(channelInputs.get(i).pushedConfigFault);
    }
  }

  public Rotation2d getRobotYaw() {
    return robotYaw;
  }

  // Function to determine if the launcher wheels and hood are at their setpoints
  public boolean isReady() {
    boolean ready = true;
    for (ChannelIO channelIO : channelIOs) {
      ready = ready && channelIO.isAtSetpoint();
    }
    return ready;
  }
}
