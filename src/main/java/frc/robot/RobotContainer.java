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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.DriveCharacterizationCommands;
import frc.robot.commands.LaunchCommands;
import frc.robot.commands.pipeline.DriveInput;
import frc.robot.commands.pipeline.DriveInputPipeline;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperRunMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRunMode;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherRunMode;
import frc.robot.subsystems.launcher.LauncherControlManual;
import frc.robot.subsystems.launcher.LauncherControlManual.ManualLaunchMode;
import frc.robot.subsystems.led.BlinkenLEDPattern;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.utility.Elastic;
import frc.robot.utility.Elastic.Notification.NotificationLevel;
import frc.robot.utility.HubTracker;
import frc.robot.utility.geometry.FieldFlipUtil;
import java.util.LinkedHashMap;
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
  private final Launcher launcher;
  private final Intake intake;

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
    launcher = Launcher.create(robotType);

    // Vision setup
    if (Constants.isOnPlayingField()) {
      vision.setAprilTagFieldLayout(FieldConstants.apriltagLayout);
      setupInitPose();
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

    registerNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", createSendableChooser());
    autoChooser.addDefaultOption("None", Commands.none());

    leds.setDefaultCommand(
        leds.runColor(
                BlinkenLEDPattern.COLORWAVES_OCEAN,
                BlinkenLEDPattern.COLORWAVES_LAVA,
                BlinkenLEDPattern.WHITE)
            .withName("LED Alliance Color Waves"));

    launcher.configure(drive::getRobotPose, drive::getRobotSpeeds);

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
            new DriveInput()
                .linearVelocityStick(
                    -xbox.getLeftY(), -xbox.getLeftX(), drive.getMaxLinearSpeedMetersPerSec())
                .angularVelocityStick(-xbox.getRightX(), drive.getMaxAngularSpeedRadPerSec())
                .fieldRelativeEnabled();

    final DriveInputPipeline pipeline = new DriveInputPipeline(drive, baseDrive);

    // Default command, normal joystick drive

    final Command aimDrive = LaunchCommands.driveWhileLaunching(drive, pipeline::getChassisSpeeds);

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
          } else if (current == aimDrive) {
            return "Aim[" + pipeline.getLayerInfo() + "]";
          } else if (current != null) {
            return current.getName();
          } else if (DriverStation.isDisabled()) {
            return "Disabled";
          }
          return "Idle";
        };

    // Toggle robot relative mode, used as backup if gyro fails
    xbox.back()
        .debounce(0.1)
        .toggleOnTrue(pipeline.runLayer("Robot Relative", DriveInput::fieldRelativeDisabled));

    // Secondary drive command, use driving stick to control angle as well
    xbox.leftTrigger().whileTrue(pipeline.runLayer("Intake", DriveInput::locustMode));

    // Slow mode, reduce translation and rotation speeds for fine control
    xbox.leftBumper()
        .whileTrue(
            pipeline.runLayer(
                "Slow", input -> input.linearCoefficient(0.3).angularCoefficient(0.3)));

    xbox.rightTrigger().whileTrue(aimDrive);

    // Secondary drive command, right stick will be used to control target angular
    // position instead of angular velocity
    xbox.rightBumper()
        .whileTrue(
            pipeline.runLayer(
                "Heading", input -> input.headingStick(-xbox.getRightY(), -xbox.getRightX())));

    xbox.rightBumper()
        .multiPress(2, 0.05)
        .toggleOnTrue(pipeline.runLayer("Hold", DriveInput::passiveHoldHeading));

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
                  input
                      .linearVelocity(translation)
                      .fieldRelativeDisabled()
                      .angularCoefficient(0.3));
      xbox.pov(pov).whileTrue(activateLayer);
    }
  }

  private void configureOperatorControllerBindings(CommandXboxController xbox) {

    final LauncherControlManual manualLaunchControl = new LauncherControlManual(ManualLaunchMode.Y);

    launcher.setManualModeState(manualLaunchControl);

    final LauncherRunMode DEFAULT_LAUNCH = LauncherRunMode.INTERPOLATION;

    launcher.setMode(DEFAULT_LAUNCH);

    final Trigger manualButton = xbox.back();
    final Trigger resetButton = xbox.start().debounce(0.01);

    // --- INTAKE CONTROL ---

    // Intake button (hold)
    xbox.leftTrigger()
        .whileTrue(
            Commands.parallel(
                    intake.run(() -> intake.setMode(IntakeRunMode.INTAKING)),
                    hopper.run(() -> hopper.setMode(HopperRunMode.IDLE)))
                .finallyDo(
                    () -> {
                      intake.setMode(IntakeRunMode.UP);
                      hopper.setMode(HopperRunMode.STOPPED);
                    })
                .withName("Intake"));

    // Dump through intake button (hold)
    xbox.leftBumper()
        .debounce(0.1)
        .whileTrue(
            Commands.parallel(
                    intake.run(() -> intake.setMode(IntakeRunMode.OUTTAKING)),
                    hopper.run(() -> hopper.setMode(HopperRunMode.REVERSE)))
                .finallyDo(
                    () -> {
                      intake.setMode(IntakeRunMode.UP);
                      hopper.setMode(HopperRunMode.STOPPED);
                    })
                .withName("Intake"));

    // Deploy intake tap button
    xbox.leftStick()
        .onTrue(
            intake
                .runOnce(() -> intake.setModeNoWheels(IntakeRunMode.INTAKING))
                .withName("Deploy intake"));

    // Retract intake tap button
    xbox.rightStick()
        .onTrue(
            intake
                .runOnce(() -> intake.setModeNoWheels(IntakeRunMode.UP))
                .withName("Retract intake"));

    // Intake shift up button
    xbox.povUp()
        .and(manualButton.negate())
        .onTrue(
            Commands.runOnce(() -> intake.shiftSetpoint(Rotation2d.fromDegrees(+1)))
                .withName("Shift intake up"));

    // Intake shift down button
    xbox.povDown()
        .and(manualButton.negate())
        .onTrue(
            Commands.runOnce(() -> intake.shiftSetpoint(Rotation2d.fromDegrees(-1)))
                .withName("Shift intake down"));

    // Reset intake shift button
    resetButton
        .and(manualButton.negate())
        .onTrue(Commands.runOnce(intake::unshiftSetpoint).withName("Reset intake shift"));

    // --- OUTTAKE CONTROL ---

    // Spin up then launch button
    xbox.rightTrigger()
        .whileTrue(
            launcher
                .runOnce(launcher::start)
                .andThen(launcher.idle().until(launcher::atGoalDebounced))
                .andThen(hopper.runOnce(() -> hopper.setMode(HopperRunMode.FIRING)))
                .andThen(launcher.idle())
                .finallyDo(
                    () -> {
                      launcher.stop();
                      hopper.setMode(HopperRunMode.STOPPED);
                    })
                .withName("Spin up and Launch"));

    // Force launch button
    xbox.rightBumper()
        .whileTrue(
            launcher
                .run(launcher::start)
                .alongWith(hopper.run(() -> hopper.setMode(HopperRunMode.FIRING)))
                .finallyDo(
                    () -> {
                      launcher.stop();
                      hopper.setMode(HopperRunMode.STOPPED);
                    })
                .withName("Force Launch"));

    // --- LAUNCH PREP CONTROLS ---

    // Start spin up button
    xbox.y()
        .and(manualButton.negate())
        .onTrue(launcher.runOnce(launcher::start).withName("Spin Up"));

    // Cancel spin up button
    xbox.b()
        .and(manualButton.negate())
        .onTrue(
            launcher
                .runOnce(launcher::stop)
                .andThen(hopper.runOnce(() -> hopper.setMode(HopperRunMode.STOPPED)))
                .withName("Cancel Spin Up"));

    // Reverse lifter & outtake button
    xbox.x()
        .and(manualButton.negate())
        .whileTrue(
            Commands.parallel(
                    launcher.run(() -> launcher.setDutyCycle(-0.1)),
                    hopper.run(() -> hopper.setMode(HopperRunMode.PREP_SHOT)))
                .finallyDo(
                    () -> {
                      launcher.stop();
                      hopper.setMode(HopperRunMode.STOPPED);
                    })
                .withName("Reverse Launcher and Hopper"));

    // Reverse lifter button
    xbox.a()
        .and(manualButton.negate())
        .whileTrue(
            hopper
                .run(() -> hopper.setMode(HopperRunMode.PREP_SHOT))
                .finallyDo(() -> hopper.setMode(HopperRunMode.STOPPED))
                .withName("Reverse Hopper"));

    // --- MANUAL LAUNCH MODE CONTROLS ---

    final Trigger anyManualModeLetterButton = xbox.a().or(xbox.b()).or(xbox.x()).or(xbox.y());
    final Trigger isManualMode = new Trigger(() -> launcher.getMode() == LauncherRunMode.MANUAL);

    // Manual mode is turned on when preset is chosen
    manualButton
        .and(anyManualModeLetterButton)
        .onTrue(
            Commands.runOnce(() -> launcher.setMode(LauncherRunMode.MANUAL))
                .ignoringDisable(true)
                .withName("Manual Launch Mode"));

    // Manual mode is turned off on double tap
    manualButton
        .multiPress(2, 0.3)
        .and(anyManualModeLetterButton.negate())
        .onFalse(
            Commands.runOnce(() -> launcher.setMode(DEFAULT_LAUNCH))
                .ignoringDisable(true)
                .withName("Default Launch Mode"));

    // Manual mode preset buttons
    xbox.y().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.Y));
    xbox.x().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.X));
    xbox.a().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.A));
    xbox.b().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.B));

    // Manual mode preset adjustment buttons
    // These have no overlap so can be turned on whenever manual mode is active
    xbox.povRight()
        .and(manualButton.or(isManualMode))
        .onTrue(manualLaunchControl.incrementHoodCommand(+0.025));
    xbox.povLeft()
        .and(manualButton.or(isManualMode))
        .onTrue(manualLaunchControl.incrementHoodCommand(-0.025));

    // Manual mode preset adjustment buttons
    xbox.povUp().and(manualButton).onTrue(manualLaunchControl.incrementVelocityCommand(+20));
    xbox.povDown().and(manualButton).onTrue(manualLaunchControl.incrementVelocityCommand(-20));
    resetButton.and(manualButton).onTrue(manualLaunchControl.resetCommand());

    // --- HANG/MANUAL CONTROL ---

    // Hang up/down axis
    xbox.getRightY();
  }

  private Command rumbleController(
      CommandXboxController controller, double rumbleIntensity, RumbleType type) {
    return Commands.startEnd(
            () -> controller.setRumble(type, rumbleIntensity), () -> controller.setRumble(type, 0))
        .withName("Rumble Controller " + controller.getHID().getPort());
  }

  private Command rumbleController(CommandXboxController controller, double rumbleIntensity) {
    return rumbleController(controller, rumbleIntensity, RumbleType.kBothRumble);
  }

  private Command rumbleControllers(double rumbleIntensity) {
    return Commands.parallel(
            rumbleController(driverController, rumbleIntensity),
            rumbleController(operatorController, rumbleIntensity))
        .withName("Rumble Both Controllers");
  }

  private void configureAlertTriggers() {
    new Trigger(HubTracker::isActive).onChange(rumbleControllers(1.0).withTimeout(0.25));

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
    LinkedHashMap<String, Command> namedCommands = new LinkedHashMap<String, Command>();

    namedCommands.put("LEDS", leds.runColor(BlinkenLEDPattern.RED));

    // Hopper commands
    namedCommands.put("StopHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.STOPPED)));
    namedCommands.put("IdleHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.IDLE)));
    namedCommands.put("FireHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.FIRING)));
    namedCommands.put("ReverseHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.REVERSE)));

    // Intake commands
    namedCommands.put(
        "ExtendSlapdown", intake.runOnce(() -> intake.setModeNoWheels(IntakeRunMode.INTAKING)));
    namedCommands.put(
        "RetractSlapdown", intake.runOnce(() -> intake.setModeNoWheels(IntakeRunMode.UP)));
    namedCommands.put("StartIntake", intake.runOnce(() -> intake.setMode(IntakeRunMode.INTAKING)));
    namedCommands.put("StopIntake", intake.runOnce(() -> intake.setMode(IntakeRunMode.UP)));

    // Launcher commands
    namedCommands.put("PrimeToLaunch", LaunchCommands.primeToLaunch(drive, launcher));
    namedCommands.put("LaunchInPlace", Commands.none());

    // Hang commands
    namedCommands.put("HangUp", Commands.none());
    namedCommands.put("HangDown", Commands.none());

    System.out.println("Named commands:");
    for (var commandName : namedCommands.keySet()) {
      try {
        NamedCommands.registerCommand(commandName, namedCommands.get(commandName));
        System.out.println("[ OK ] " + commandName);
      } catch (Exception e) {
        System.err.println("[FAIL] " + commandName);
      }
    }
    System.out.println("Named commands registered.");
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

  private SendableChooser<Command> createSendableChooser() {
    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
    // Choreo Autos
    // https://pathplanner.dev/pplib-choreo-interop.html#load-choreo-trajectory-as-a-pathplannerpath

    var chooser =
        Constants.DEVELOPMENT_MODE
            ? AutoBuilder.buildAutoChooser()
            : new SendableChooser<Command>();

    if (Constants.DEVELOPMENT_MODE) {
      chooser.addOption(
          "[Characterization] Drive Feed Forward",
          DriveCharacterizationCommands.feedforwardCharacterization(drive));
      chooser.addOption(
          "[Characterization] Drive Wheel Radius",
          DriveCharacterizationCommands.wheelRadiusCharacterization(drive));

      chooser.addOption(
          "[SysId] Drive Quasistatic Forward",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "[SysId] Drive Quasistatic Reverse",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      chooser.addOption(
          "[SysId] Drive Dynamic Forward", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "[SysId] Drive Dynamic Reverse", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    } else {
      chooser.addOption("Nothing", Commands.none()); // TODO Add useful autos
    }

    return chooser;
  }

  private void setupInitPose() {
    Pose2d startingPose =
        new Pose2d(
            FieldConstants.LinesVertical.starting,
            FieldConstants.fieldWidth / 2.0,
            Rotation2d.kZero);

    Pose2d staringPoseFallback =
        new Pose2d(
            new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth).div(2),
            Rotation2d.kCW_90deg);

    drive.resetPose(staringPoseFallback);

    CommandScheduler.getInstance()
        .schedule(
            Commands.waitUntil(() -> DriverStation.getAlliance().isPresent())
                .andThen(Commands.runOnce(() -> drive.resetPose(FieldFlipUtil.apply(startingPose))))
                .withTimeout(3)
                .onlyWhile(() -> drive.getRobotPose() == staringPoseFallback)
                .ignoringDisable(true)
                .withName("Init Pose"));
  }
}
