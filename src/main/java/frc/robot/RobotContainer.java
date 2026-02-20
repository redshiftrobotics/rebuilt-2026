package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.DriveCharacterizationCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.pipeline.DriveInput;
import frc.robot.commands.pipeline.DriveInputPipeline;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.RunMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.led.BlinkenLEDPattern;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.utility.Elastic;
import frc.robot.utility.Elastic.Notification.NotificationLevel;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
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
  private final Hopper hopper;
  private final Intake intake;
  private final Climb climb;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Alerts
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
  private final Alert notPrimaryBotAlert =
      new Alert("Robot type is not the primary robot type.", AlertType.kInfo);
  private final Alert developmentModeActiveAlert =
      new Alert("Development mode active, do not use in competition.", AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** Which robot are we running on? */
  private final RobotType robotType;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    robotType = Constants.getRobot();

    System.out.println("Initializing for robot ID: " + robotType);

    drive = Drive.create(robotType);
    vision = AprilTagVision.create(robotType, drive);
    leds = LEDSubsystem.create(robotType);
    hopper = Hopper.create(robotType);
    intake = Intake.create(robotType);
    climb = Climb.create(robotType);

    // Vision setup
    if (Constants.isOnPlayingField()) {
      vision.setAprilTagFieldLayout(FieldConstants.apriltagLayout);
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

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to
    // auto populate
    registerNamedCommands();
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser",
            Constants.DEVELOPMENT_MODE
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
    developmentModeActiveAlert.set(Constants.DEVELOPMENT_MODE);
    notPrimaryBotAlert.set(Constants.getRobot() != Constants.PRIMARY_ROBOT_TYPE);

    // Hide controller missing warnings for sim
    DriverStation.silenceJoystickConnectionWarning(Constants.getMode() != Mode.REAL);

    initDashboard();

    // Configure the button bindings
    configureDriverControllerBindings(driverController);
    configureOperatorControllerBindings(operatorController);
    configureAlertTriggers();

    System.out.println(robotType + " ready.");
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

    DriverDashboard.poseSupplier = drive::getRobotPose;
    DriverDashboard.speedsSupplier = drive::getRobotSpeeds;
    DriverDashboard.wheelStatesSupplier = drive::getWheelSpeeds;
    DriverDashboard.hasVisionEstimate = vision::hasSuccessfulEstimate;
    DriverDashboard.currentHopperRunModeNameSupplier = () -> hopper.getCurrentRunMode().toString();
    DriverDashboard.hopperBubblerVelocitySupplier = hopper::getBubblerCharacterizationVelocity;
    DriverDashboard.hopperFeederVelocitySupplier = hopper::getFeederCharacterizationVelocity;

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
                    new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth).div(2),
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
    Supplier<DriveInput> baseDrive =
        () ->
            new DriveInput(drive)
                .linearVelocityStick(-xbox.getLeftY(), -xbox.getLeftX())
                .angularVelocityStick(-xbox.getRightX())
                .fieldRelativeEnabled();

    final DriveInputPipeline pipeline = new DriveInputPipeline(drive, baseDrive);

    // Default command, normal joystick drive
    drive.setDefaultCommand(
        drive
            .run(() -> drive.setRobotSpeeds(pipeline.getChassisSpeeds()))
            .finallyDo(drive::stop)
            .withName("Pipeline Drive"));

    DriverDashboard.currentDriveModeName =
        () -> {
          Command current = drive.getCurrentCommand();
          if (current == drive.getDefaultCommand()) {
            return "[" + pipeline.getLayerInfo() + "]";
          } else if (current != null) {
            return current.getName();
          }
          return "Idle";
        };

    // Toggle robot relative mode, used as backup if gyro fails
    xbox.y().toggleOnTrue(pipeline.runLayer("Robot Relative", DriveInput::fieldRelativeDisabled));

    // Secondary drive command, right stick will be used to control target angular
    // position instead of angular velocity
    // xbox.rightBumper()
    //     .whileTrue(
    //         pipeline.runLayer(
    //             "Heading Controlled",
    //             input -> input.headingStick(-xbox.getRightY(), -xbox.getRightX())));

    // Secondary drive command, use driving stick to control angle as well
    xbox.rightBumper().whileTrue(pipeline.runLayer("Locust", DriveInput::locust));

    // Slow mode, reduce translation and rotation speeds for fine control
    xbox.leftBumper()
        .whileTrue(pipeline.runLayer("Slow Mode", input -> input.coefficients(0.3, 0.3)));

    // Cause the robot to resist movement by forming an X shape with the swerve
    // modules. Helps prevent getting pushed around
    xbox.x().whileTrue(drive.run(drive::stopUsingBrakeArrangement).withName("Hold Position"));

    // Stop the robot and cancel any running commands
    xbox.b()
        .or(RobotModeTriggers.disabled())
        .onTrue(drive.runOnce(drive::stop).withName("Cancel"))
        .onTrue(rumbleControllers(0).withTimeout(0.02));

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
          pipeline.runLayer(
              String.format(
                  "Strafe %.0f", MathUtil.inputModulus(rotation.getDegrees(), -180, +180)),
              input ->
                  input.linearVelocity(translation).fieldRelativeDisabled().coefficients(1, 0.3));
      xbox.pov(pov).whileTrue(activateLayer);
    }
  }

  private void configureOperatorControllerBindings(CommandXboxController xbox) {

    xbox.leftTrigger()
        .onTrue(
            Commands.sequence(
                IntakeCommands.extendSlapdown(intake), IntakeCommands.startIntake(intake)))
        .onFalse(
            Commands.sequence(
                IntakeCommands.retractSlapdown(intake), IntakeCommands.stopIntake(intake)));

    // down pos
    xbox.leftTrigger()
        .and(xbox.pov(0))
        .onTrue(
            IntakeCommands.incrementDownSlapdown(
                intake, IntakeConstants.SLAPDOWN_INCREMENT_SETPOINT));
    xbox.leftTrigger()
        .and(xbox.pov(180))
        .onTrue(
            IntakeCommands.incrementDownSlapdown(
                intake, IntakeConstants.SLAPDOWN_INCREMENT_SETPOINT.unaryMinus()));

    // up pos
    xbox.leftTrigger()
        .negate()
        .and(xbox.pov(0))
        .onTrue(
            IntakeCommands.incrementUpSlapdown(
                intake, IntakeConstants.SLAPDOWN_INCREMENT_SETPOINT));
    xbox.leftTrigger()
        .negate()
        .and(xbox.pov(180))
        .onTrue(
            IntakeCommands.incrementUpSlapdown(
                intake, IntakeConstants.SLAPDOWN_INCREMENT_SETPOINT.unaryMinus()));

    // This is the input for firing; when the shooter is added, it should be
    // triggered by this as
    // well
    xbox.rightTrigger()
        .whileTrue(HopperCommands.setHopperMode(hopper, RunMode.FIRING))
        .onFalse(HopperCommands.setHopperMode(hopper, RunMode.STOPPED));

    // Run the bubbler at low speed to send fuel towards the back without firing
    xbox.b()
        .whileTrue(HopperCommands.setHopperMode(hopper, RunMode.FUEL_STORE))
        .onFalse(HopperCommands.setHopperMode(hopper, RunMode.STOPPED));

    // Run the hopper motors in reverse to deal with jams
    xbox.start()
        .whileTrue(HopperCommands.setHopperMode(hopper, RunMode.REVERSE))
        .onFalse(HopperCommands.setHopperMode(hopper, RunMode.STOPPED));

    climb.setDefaultCommand(
        Commands.run(() -> climb.setSpeed(MathUtil.applyDeadband(xbox.getLeftY(), 0.1)), climb));
  }

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

  /** Make commands accessible to PathPlanner autos. */
  private void registerNamedCommands() {
    Map<String, Command> namedCommands = new HashMap<String, Command>();

    namedCommands.put("LEDS", leds.runColor(BlinkenLEDPattern.RED));

    // Hopper commands
    namedCommands.put("StopHopper", HopperCommands.setHopperMode(hopper, RunMode.STOPPED));
    namedCommands.put("IdleHopper", HopperCommands.setHopperMode(hopper, RunMode.FUEL_STORE));
    namedCommands.put("FireHopper", HopperCommands.setHopperMode(hopper, RunMode.FIRING));
    namedCommands.put("ReverseHopper", HopperCommands.setHopperMode(hopper, RunMode.REVERSE));

    // Intake commands
    namedCommands.put("ExtendSlapdown", IntakeCommands.extendSlapdown(intake));
    namedCommands.put("RetractSlapdown", IntakeCommands.retractSlapdown(intake));
    namedCommands.put("StartIntake", IntakeCommands.startIntake(intake));
    namedCommands.put("StopIntake", IntakeCommands.stopIntake(intake));

    // Launcher commands

    // Hang commands
    namedCommands.put("HangUp", null);
    namedCommands.put("HangDown", null);

    System.out.println("Avaliable named commands:");
    namedCommands.keySet().forEach(commandName -> System.out.println("  " + commandName));

    NamedCommands.registerCommands(namedCommands);
  }

  private void configureAutos(LoggedDashboardChooser<Command> dashboardChooser) {
    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos

    // Choreo Autos
    // https://pathplanner.dev/pplib-choreo-interop.html#load-choreo-trajectory-as-a-pathplannerpath

    if (Constants.DEVELOPMENT_MODE) {
      dashboardChooser.addOption(
          "[Characterization] Drive Feed Forward",
          DriveCharacterizationCommands.feedforwardCharacterization(drive));
      dashboardChooser.addOption(
          "[Characterization] Drive Wheel Radius",
          DriveCharacterizationCommands.wheelRadiusCharacterization(drive));

      dashboardChooser.addOption(
          "[SysId] Drive Quasistatic Forward",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      dashboardChooser.addOption(
          "[SysId] Drive Quasistatic Reverse",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      dashboardChooser.addOption(
          "[SysId] Drive Dynamic Forward", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      dashboardChooser.addOption(
          "[SysId] Drive Dynamic Reverse", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      dashboardChooser.addOption(
          "[Test] Hopper Test Routine", HopperCommands.hopperTestRoutine(hopper));
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
