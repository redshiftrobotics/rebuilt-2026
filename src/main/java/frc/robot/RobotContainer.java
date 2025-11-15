package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.pipeline.DriveInput;
import frc.robot.commands.pipeline.DriveInputPipeline;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.dashboard.DriverDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleConstants;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.controllers.DrivePoseController;
import frc.robot.subsystems.led.BlinkenLEDPattern;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.led.LEDStripIOSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.utility.Elastic;
import frc.robot.utility.Elastic.Notification.NotificationLevel;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final AprilTagVision vision;
  private final LEDSubsystem leds;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Alerts for controller disconnection
  private final Alert driverDisconnected =
      new Alert(
          String.format(
              "Driver xbox controller disconnected (port %s).",
              driverController.getHID().getPort()),
          AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert(
          String.format(
              "Operator xbox controller disconnected (port %s).",
              operatorController.getHID().getPort()),
          AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final Alert notPrimaryBotAlert =
      new Alert("Robot type is not the primary robot type.", AlertType.kInfo);
  private final Alert tuningModeActiveAlert =
      new Alert("Tuning mode active, do not use in competition.", AlertType.kWarning);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case PRESEASON_2026:
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID, true),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision = new AprilTagVision(drive::getRobotPose);
        leds = new LEDSubsystem();
        break;

      case CHASSIS_CANNON:
      case WOOD_BOT_2026:
      case REEFSCAPE_2025:
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID, false),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision(drive::getRobotPose);
        leds = new LEDSubsystem();
        break;

      case SIM_BOT:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new AprilTagVision(
                drive::getRobotPose, new CameraIOSim(VisionConstants.SIM_FRONT_CAMERA));
        leds = new LEDSubsystem(new LEDStripIOSim(LEDConstants.DEFAULT_PATTERN));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new AprilTagVision(drive::getRobotPose);
        leds = new LEDSubsystem();
        break;
    }

    // Vision setup
    if (Constants.isOnPlayingField()) {
      vision.setAprilTagFieldLayout(FieldConstants.FIELD_APRIL_TAGS);
    }

    vision.setVisionPoseConsumer(
        (estimate) -> {
          if (estimate.status().isSuccess() && Constants.getMode() != Mode.SIM) {
            drive.addVisionMeasurement(
                estimate.estimatedPose().toPose2d(),
                estimate.timestampSeconds(),
                estimate.standardDeviations());
          }
        });

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to auto populate
    registerNamedCommands();
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser",
            Constants.INCLUDE_ALL_PATHPLANNER_AUTOS
                ? AutoBuilder.buildAutoChooser()
                : new SendableChooser<Command>());
    autoChooser.addDefaultOption("None", Commands.none());

    // Configure autos
    configureAutos(autoChooser);

    leds.setDefaultCommand(
        leds.runColor(
                BlinkenLEDPattern.COLORWAVES_OCEAN,
                BlinkenLEDPattern.COLORWAVES_LAVA,
                BlinkenLEDPattern.WHITE)
            .withName("LED Alliance Color Waves"));

    // Alerts for constants to avoid using them in competition
    tuningModeActiveAlert.set(Constants.TUNING_MODE);
    notPrimaryBotAlert.set(Constants.getRobot() != Constants.PRIMARY_ROBOT_TYPE);

    // Hide controller missing warnings for sim
    DriverStation.silenceJoystickConnectionWarning(Constants.getMode() != Mode.REAL);

    initDashboard();

    // Configure the button bindings
    configureDriverControllerBindings(driverController);
    configureOperatorControllerBindings(operatorController);
    configureAlertTriggers();
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

    DriverDashboard.poseSupplier = drive::getRobotPose;
    DriverDashboard.speedsSupplier = drive::getRobotSpeeds;
    DriverDashboard.wheelStatesSupplier = drive::getWheelSpeeds;
    DriverDashboard.hasVisionEstimate = vision::hasSuccessfulEstimate;

    DriverDashboard.currentDriveModeName =
        () -> drive.getCurrentCommand() == null ? "Idle" : drive.getCurrentCommand().getName();

    DriverDashboard.addCommand("Reset Pose", () -> drive.resetPose(new Pose2d()), true);
    DriverDashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () ->
                drive.resetPose(
                    new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero))),
        true);
    DriverDashboard.addCommand(
        "Reset Centered",
        () ->
            drive.resetPose(
                new Pose2d(
                    FieldConstants.FIELD_CORNER_TO_CORNER.div(2),
                    drive.getRobotPose().getRotation())),
        true);
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  }

  private void configureDriverControllerBindings(CommandXboxController xbox) {

    final DriveInputPipeline pipeline = new DriveInputPipeline(drive);

    // Default command, normal joystick drive
    drive.setDefaultCommand(
        drive
            .run(() -> drive.setRobotSpeeds(pipeline.getChassisSpeeds()))
            .finallyDo(drive::stop)
            .withName("Pipeline Drive"));

    RobotModeTriggers.teleop()
        .whileTrue(
            pipeline.activatePermanentLayer(
                "Drive",
                input ->
                    input
                        .linearVelocityStick(-xbox.getLeftY(), -xbox.getLeftX())
                        .angularVelocityStick(-xbox.getRightX())
                        .fieldRelativeEnabled()));

    DriverDashboard.currentDriveModeName =
        () -> {
          Command current = drive.getCurrentCommand();
          if (current == drive.getDefaultCommand()) {
            String layers = String.join(" → ", pipeline.getActiveLayers());
            return "[" + layers + "]";
          } else if (current != null) {
            return current.getName();
          }
          return "Idle";
        };

    // Toggle robot relative mode, used as backup if gyro fails
    xbox.y()
        .toggleOnTrue(
            pipeline.activateLayer("Robot Relative", input -> input.fieldRelativeDisabled()));

    // Secondary drive command, right stick will be used to control target angular position instead
    // of angular velocity
    xbox.rightBumper()
        .whileTrue(
            pipeline.activateLayer(
                "Heading Controlled",
                input -> input.headingStick(-xbox.getRightY(), -xbox.getRightX())));

    // Slow mode, reduce translation and rotation speeds for fine control
    xbox.leftBumper()
        .whileTrue(
            pipeline.activateLayer(
                "Slow Mode",
                input -> input.linearVelocityCoefficient(0.3).angularVelocityCoefficient(0.3)));

    // Cause the robot to resist movement by forming an X shape with the swerve modules
    // Helps prevent getting pushed around
    xbox.x().whileTrue(drive.run(drive::stopUsingBrakeArrangement).withName("Hold Position"));

    // Stop the robot and cancel any running commands
    xbox.b()
        .or(RobotModeTriggers.disabled())
        .onTrue(drive.runOnce(drive::stop).withName("Cancel"))
        .onTrue(rumbleControllers(0).withTimeout(Constants.LOOP_PERIOD_SECONDS));

    xbox.b()
        .debounce(1)
        .onTrue(rumbleController(xbox, 0.3).withTimeout(0.25))
        .whileTrue(drive.run(drive::stopUsingForwardArrangement).withName("Stop and Orient"));

    // Reset the gyro heading
    xbox.start()
        .debounce(0.3)
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero)))
                .andThen(rumbleController(xbox, 0.3).withTimeout(0.25))
                .ignoringDisable(true)
                .withName("Reset Gyro Heading"));

    // Configure the driving dpad
    for (int pov = 0; pov < 360; pov += 45) {
      Rotation2d rotation = Rotation2d.fromDegrees(-pov);
      Translation2d translation = new Translation2d(1, rotation);
      Command activateLayer =
          pipeline.activateLayer(
              String.format("Strafe %.0f°", translation.getAngle().getDegrees()),
              input ->
                  input
                      .linearVelocity(translation)
                      .fieldRelativeDisabled()
                      .angularVelocityCoefficient(0.3));
      xbox.pov(pov).whileTrue(activateLayer);
    }

    if (Constants.isDemoMode()) {

      DrivePoseController poseController = drive.getPoseController();

      UnaryOperator<DriveInput> aim =
          input ->
              input.facingPoint(
                  drive.getPoseController().getNextGoal().map(Pose2d::getTranslation).orElse(null));

      xbox.a()
          .whileTrue(
              pipeline
                  .activateLayer("Face Setpoint", aim)
                  .onlyIf(() -> poseController.getNextGoal().isPresent()));

      // Drive to pose setpoint reset
      RobotModeTriggers.disabled()
          .onTrue(Commands.runOnce(poseController::reset).withName("Reset Pose Controller"));

      // Save current pose as setpoint
      xbox.leftTrigger()
          .onTrue(rumbleController(xbox, 0.4).withTimeout(0.1))
          .onTrue(
              Commands.runOnce(
                      () -> {
                        Pose2d setpoint = drive.getRobotPose();
                        poseController.setSetpoint(setpoint);
                        Logger.recordOutput("Teleop/PoseGoal", setpoint);
                      })
                  .withName("Save Setpoint"));

      // Drive to pose setpoint
      xbox.rightTrigger()
          .whileTrue(
              drive
                  .run(() -> drive.setRobotSpeeds(poseController.calculate()))
                  .finallyDo(drive::stop)
                  .beforeStarting(poseController::reset)
                  .alongWith(rumbleController(xbox, 0.1, RumbleType.kLeftRumble))
                  .until(poseController::atGoal)
                  .andThen(rumbleController(xbox, 1, RumbleType.kRightRumble).withTimeout(0.2))
                  .onlyIf(() -> poseController.getNextGoal().isPresent())
                  .withName("Drive to Setpoint"));
    }
  }

  private void configureOperatorControllerBindings(CommandXboxController xbox) {}

  private Command rumbleController(
      CommandXboxController controller, double rumbleIntensity, RumbleType type) {
    return Commands.startEnd(
            () -> controller.setRumble(type, rumbleIntensity), () -> controller.setRumble(type, 0))
        .withName("RumbleController");
  }

  private Command rumbleController(CommandXboxController controller, double rumbleIntensity) {
    return rumbleController(controller, rumbleIntensity, RumbleType.kBothRumble);
  }

  private Command rumbleControllers(double rumbleIntensity) {
    return Commands.parallel(
        rumbleController(driverController, rumbleIntensity),
        rumbleController(operatorController, rumbleIntensity));
  }

  private void configureAlertTriggers() {
    // Endgame alert triggers
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 20)
        .onTrue(rumbleControllers(0.5).withTimeout(0.5));

    RobotModeTriggers.teleop()
        .and(RobotBase::isReal)
        .onChange(rumbleControllers(0.2).withTimeout(0.2));

    Trigger isMatch = new Trigger(() -> DriverStation.getMatchTime() != -1);

    RobotModeTriggers.teleop()
        .and(isMatch)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Teleoperated")));

    RobotModeTriggers.autonomous()
        .and(isMatch)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Autonomous")));
  }

  private void registerNamedCommands() {
    // Set up named commands for path planner auto
    NamedCommands.registerCommand("LEDS", leds.runColor(BlinkenLEDPattern.RED));
  }

  private void configureAutos(LoggedDashboardChooser<Command> dashboardChooser) {

    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos

    // Choreo Autos
    // https://pathplanner.dev/pplib-choreo-interop.html#load-choreo-trajectory-as-a-pathplannerpath

    if (Constants.RUNNING_TEST_PLANS) {
      dashboardChooser.addOption(
          "[Characterization] Drive Feed Forward",
          DriveCommands.feedforwardCharacterization(drive));
      dashboardChooser.addOption(
          "[Characterization] Drive Wheel Radius",
          DriveCommands.wheelRadiusCharacterization(drive));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Constants.isDemoMode() && !Constants.isOnPlayingField()) {
      Elastic.sendNotification(
          new Elastic.Notification(
              NotificationLevel.WARNING,
              "Demo mode off field: auto disabled",
              "Autonomous command disabled in demo mode when not on playing field and in demo mode. Check Constants.java"));
      return null;
    }
    return autoChooser.get();
  }
}
