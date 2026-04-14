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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
import frc.robot.commands.SelfDrivingCommands;
import frc.robot.commands.StagedAgitateFeed;
import frc.robot.commands.pipeline.DriveInput;
import frc.robot.commands.pipeline.DriveInputPipeline;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperRunMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRunMode;
import frc.robot.subsystems.launcher.LaunchCalculator;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherRunMode;
import frc.robot.subsystems.launcher.LauncherControlManual;
import frc.robot.subsystems.launcher.LauncherControlManual.ManualLaunchMode;
import frc.robot.subsystems.led.BlinkinLEDPattern;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.utility.Elastic;
import frc.robot.utility.Elastic.Notification.NotificationLevel;
import frc.robot.utility.HubShiftUtil;
import frc.robot.utility.HubShiftUtil.ShiftEnum;
import frc.robot.utility.HubShiftUtil.ShiftInfo;
import frc.robot.utility.geometry.FieldFlipUtil;
import java.util.LinkedHashMap;
import java.util.Set;
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
    private final CommandXboxController operatorController =
            Constants.isDemoModeOneController() ? null : new CommandXboxController(1);

    // Alerts
    private final Alert driverDisconnected = new Alert(
            String.format(
                    "Driver xbox controller disconnected (port %s).",
                    driverController.getHID().getPort()),
            AlertType.kWarning);
    private final Alert operatorDisconnected = new Alert(
            String.format(
                    "Operator xbox controller disconnected (port %s).",
                    operatorController != null ? operatorController.getHID().getPort() : "N/A"),
            AlertType.kWarning);

    private final Alert notPrimaryBotAlert = new Alert("Robot type is not the primary robot type.", AlertType.kInfo);
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
        }
        setupInitPose();

        vision.setVisionPoseConsumer((estimate) -> {
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

        launcher.configure(drive::getRobotPose, drive::getRobotSpeeds);

        // Alerts for constants to avoid using them in competition
        developmentModeActiveAlert.set(Constants.DEVELOPMENT_MODE);
        notPrimaryBotAlert.set(Constants.getRobot() != Constants.PRIMARY_ROBOT_TYPE);

        // Hide controller missing warnings for sim
        DriverStation.silenceJoystickConnectionWarning(Constants.getMode() != Mode.REAL);

        initDashboard();

        // Configure the button bindings
        configureAlertTriggers();
        if (Constants.isDemoMode()) {
            configureDemoLEDs();
        } else {
            configureLEDs();
        }

        if (Constants.isDemoModeOneController()) {
            configureDemoControllerBindings(driverController);
        } else {
            configureDriverControllerBindings(driverController);
            configureOperatorControllerBindings(operatorController);
        }

        System.out.println(robotType + " ready.");
    }

    /** Configure drive dashboard object */
    private void initDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

        DriverDashboard.poseSupplier = drive::getRobotPose;
        DriverDashboard.speedsSupplier = drive::getRobotSpeeds;
        DriverDashboard.wheelStatesSupplier = drive::getWheelSpeeds;
        DriverDashboard.hasVisionEstimate = vision::hasSuccessfulEstimate;

        DriverDashboard.currentDriveModeName = () -> drive.getCurrentCommand() == null
                ? "Idle"
                : drive.getCurrentCommand().getName();

        DriverDashboard.addCommand("Reset Pose", () -> drive.resetPose(new Pose2d()), true);
        DriverDashboard.addCommand(
                "Reset Rotation",
                drive.runOnce(
                        () -> drive.resetPose(new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero))),
                true);
        DriverDashboard.addCommand(
                "Reset Centered",
                () -> drive.resetPose(new Pose2d(
                        new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth).div(2),
                        drive.getRobotPose().getRotation())),
                true);
    }

    public void updateAlerts() {
        // Controller disconnected alerts
        driverDisconnected.set(!DriverStation.isJoystickConnected(
                        driverController.getHID().getPort())
                || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
        if (operatorController != null) {
            operatorDisconnected.set(!DriverStation.isJoystickConnected(
                            operatorController.getHID().getPort())
                    || !DriverStation.getJoystickIsXbox(
                            operatorController.getHID().getPort()));
        }
    }
    /**
     * Configures the bindings for the demo controller.
     *
     * This is a single controller configuration used for robot demonstrations.
     *
     * @param xbox The driver controller
     */
    private void configureDemoControllerBindings(CommandXboxController xbox) {

        // --- Drive Control ---

        Supplier<DriveInput> baseDrive = () -> new DriveInput()
                .linearVelocityStick(-xbox.getLeftY(), -xbox.getLeftX(), drive.getMaxLinearSpeedMetersPerSec())
                .angularVelocityStick(-xbox.getRightX(), drive.getMaxAngularSpeedRadPerSec())
                .fieldRelativeEnabled();

        final DriveInputPipeline pipeline = new DriveInputPipeline(drive, baseDrive);

        // Default command, normal joystick drive
        drive.setDefaultCommand(drive.run(() -> drive.setRobotSpeeds(pipeline.getChassisSpeeds()))
                .finallyDo(drive::stop)
                .withName("Pipeline Drive"));

        DriverDashboard.currentDriveModeName = () -> {
            Command current = drive.getCurrentCommand();
            if (current == drive.getDefaultCommand()) {
                return "[" + pipeline.getLayerInfo() + "]";
            } else if (current != null) {
                return current.getName();
            } else if (DriverStation.isDisabled()) {
                return "Disabled";
            }
            return "Idle";
        };

        // Toggle robot relative mode, used as backup if gyro fails
        xbox.back().debounce(0.1).toggleOnTrue(pipeline.runLayer("Robot Relative", DriveInput::fieldRelativeDisabled));

        // Slow mode, reduce translation and rotation speeds for fine control
        xbox.leftBumper().whileTrue(pipeline.runLayer("Slow", input -> input.linearCoefficient(0.3)
                .angularCoefficient(0.3)));

        // Secondary drive command, right stick will be used to control target angular
        // position instead of angular velocity
        xbox.rightBumper()
                .whileTrue(pipeline.runLayer(
                        "Heading", input -> input.headingStick(-xbox.getRightY(), -xbox.getRightX())));

        // Cause the robot to resist movement by forming an X shape with the swerve
        // modules. Helps prevent getting pushed around, and is good quick brake
        xbox.x()
                .onTrue(Commands.runOnce(drive::stop).withName("Stop"))
                .whileTrue(drive.run(drive::stopUsingBrakeArrangement).withName("Hold Position"))
                .onTrue(rumbleControllers(0.0, RumbleType.kBothRumble).withTimeout(0.02))
                .whileTrue(leds.runColor(BlinkinLEDPattern.OFF));

        // Reset the gyro heading
        xbox.start()
                .debounce(0.3)
                .onTrue(drive.runOnce(() ->
                                drive.resetPose(new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero)))
                        .andThen(rumbleController(xbox, 0.3, RumbleType.kLeftRumble)
                                .withTimeout(0.25))
                        .ignoringDisable(true)
                        .withName("Reset Gyro Heading"));

        // --- Launcher Shooting Control ---

        // Spin up flywheels
        xbox.leftTrigger()
                .whileTrue(launcher.startEnd(launcher::start, launcher::stop).withName("Spin up"));

        // Spin up flywheels for shooting
        xbox.rightTrigger()
                .whileTrue(launcher.startEnd(launcher::start, launcher::stop)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withName("Spin up for shoot"));

        // Shoot when ready
        xbox.rightTrigger()
                .and(launcher::isReadyDebounced)
                .or(xbox.rightTrigger(0.01).and(xbox.leftTrigger()))
                .onTrue(hopper.runEnd(
                                () -> hopper.setMode(HopperRunMode.FIRING), () -> hopper.setMode(HopperRunMode.STOPPED))
                        .alongWith(new StagedAgitateFeed(intake))
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .onlyWhile(launcher::isRunning)
                        .withName("Hopper firing when ready"));

        // --- Launcher Shot Parameters Control ---

        final LauncherControlManual manualLaunchControl = new LauncherControlManual(ManualLaunchMode.MEDIUM);
        launcher.setManualModeState(manualLaunchControl);

        final LauncherRunMode DEFAULT_LAUNCH = LauncherRunMode.MANUAL;
        launcher.setMode(DEFAULT_LAUNCH);

        final Trigger manualButton = xbox.b();
        manualButton.onTrue(manualLaunchControl.resetCommand()).onFalse(manualLaunchControl.resetCommand());

        // Manual mode preset buttons
        xbox.povRight().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.FAR));
        xbox.povLeft().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.MEDIUM));
        xbox.povUp().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.UP));
        xbox.povDown().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.LOW));

        // Manual mode preset adjustment buttons
        xbox.povRight().and(manualButton.negate()).onTrue(manualLaunchControl.incrementHoodCommand(+0.1));
        xbox.povLeft().and(manualButton.negate()).onTrue(manualLaunchControl.incrementHoodCommand(-0.1));
        xbox.povUp().and(manualButton.negate()).onTrue(manualLaunchControl.incrementVelocityCommand(+10));
        xbox.povDown().and(manualButton.negate()).onTrue(manualLaunchControl.incrementVelocityCommand(-10));

        // --- Intake Control ---

        xbox.a()
                .toggleOnTrue(intake.runEnd(
                                () -> intake.setMode(IntakeRunMode.INTAKING),
                                () -> intake.setMode(IntakeRunMode.INTAKING_NO_WHEELS))
                        .withName("Intake"));

        new Trigger(() -> intake.getMode() == IntakeRunMode.INTAKING)
                .whileTrue(hopper.runEnd(
                                () -> hopper.setMode(HopperRunMode.IDLE), () -> hopper.setMode(HopperRunMode.STOPPED))
                        .withName("Hopper Intake"));

        xbox.y()
                .onTrue(intake.runOnce(() -> intake.setMode(IntakeRunMode.POST_INTAKE_TRANSITION))
                        .andThen(Commands.waitSeconds(0.3))
                        .onlyIf(() -> intake.getMode() == IntakeRunMode.INTAKING)
                        .finallyDo(() -> intake.setMode(IntakeRunMode.UP)));
    }

    /**
     * Configures the bindings for the driver controller.
     *
     * @param xbox The driver controller
     */
    private void configureDriverControllerBindings(CommandXboxController xbox) {
        Supplier<DriveInput> baseDrive = () -> new DriveInput()
                .linearVelocityStick(-xbox.getLeftY(), -xbox.getLeftX(), drive.getMaxLinearSpeedMetersPerSec())
                .angularVelocityStick(-xbox.getRightX(), drive.getMaxAngularSpeedRadPerSec())
                .fieldRelativeEnabled();

        final DriveInputPipeline pipeline = new DriveInputPipeline(drive, baseDrive);

        // Default command, normal joystick drive

        final Command aimDrive = LaunchCommands.driveWhileLaunching(drive, vision, pipeline::getChassisSpeeds)
                .withName("Aim Drive");

        drive.setDefaultCommand(drive.run(() -> drive.setRobotSpeeds(pipeline.getChassisSpeeds()))
                .finallyDo(drive::stop)
                .withName("Pipeline Drive"));

        DriverDashboard.currentDriveModeName = () -> {
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
        xbox.back().debounce(0.1).toggleOnTrue(pipeline.runLayer("Robot Relative", DriveInput::fieldRelativeDisabled));

        // Secondary drive command, use driving stick to control angle as well
        // xbox.leftTrigger().whileTrue(pipeline.runLayer("Intake", DriveInput::locustMode));

        xbox.leftTrigger().whileTrue(launcher.startEnd(launcher::start, launcher::stop));

        xbox.rightTrigger()
                .and(xbox.y().negate())
                .whileTrue(aimDrive)
                .whileTrue(launcher.startEnd(launcher::start, launcher::stop)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withName("Aim and shoot"));

        xbox.rightTrigger()
                .and(launcher::isReadyDebounced)
                .and(() -> LaunchCommands.isDriveAtLaunchGoal(drive))
                .onTrue(hopper.runEnd(
                                () -> hopper.setMode(HopperRunMode.FIRING), () -> hopper.setMode(HopperRunMode.STOPPED))
                        .alongWith(new StagedAgitateFeed(intake))
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .onlyWhile(launcher::isRunning)
                        .withName("Hopper firing when Aim Ready"));

        // Slow mode, reduce translation and rotation speeds for fine control
        xbox.leftBumper().whileTrue(pipeline.runLayer("Slow", input -> input.linearCoefficient(0.3)
                .angularCoefficient(0.3)));

        // Secondary drive command, right stick will be used to control target angular
        // position instead of angular velocity
        xbox.rightBumper()
                .whileTrue(pipeline.runLayer(
                        "Heading", input -> input.headingStick(-xbox.getRightY(), -xbox.getRightX())));

        // Cause the robot to resist movement by forming an X shape with the swerve
        // modules. Helps prevent getting pushed around
        xbox.x().whileTrue(drive.run(drive::stopUsingBrakeArrangement).withName("Hold Position"));

        // Stop the robot and cancel any running commands
        xbox.b()
                .or(RobotModeTriggers.disabled())
                .onTrue(drive.runOnce(drive::stop).withName("Cancel"))
                .onTrue(rumbleControllers(0.0, RumbleType.kLeftRumble).withTimeout(0.02));

        xbox.b()
                .debounce(1)
                .onTrue(rumbleController(xbox, 0.3, RumbleType.kLeftRumble).withTimeout(0.25))
                .whileTrue(drive.run(drive::stopUsingForwardArrangement).withName("Stop and Orient"));

        // Reset the gyro heading
        xbox.start()
                .debounce(0.3)
                .onTrue(drive.runOnce(() ->
                                drive.resetPose(new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero)))
                        .andThen(rumbleController(xbox, 0.3, RumbleType.kLeftRumble)
                                .withTimeout(0.25))
                        .ignoringDisable(true)
                        .withName("Reset Gyro Heading"));

        xbox.y().whileTrue(SelfDrivingCommands.selfDriveToOtherZone(drive));

        // face nearest angle, forward or backward depending on whats closer
        xbox.a().whileTrue(pipeline.runLayer("Forward", input -> input.headingTarget(
                        Math.abs(drive.getRobotPose()
                                                .getRotation()
                                                .minus(Rotation2d.kZero)
                                                .getRadians())
                                        < Math.PI / 2
                                ? Rotation2d.kZero
                                : Rotation2d.k180deg)
                .linearCoefficient(0.9)));

        // Configure the driving dpad
        for (int pov = 0; pov < 360; pov += 45) {
            Rotation2d rotation = Rotation2d.fromDegrees(-pov);
            Translation2d translation = new Translation2d(1, rotation);
            Command activateLayer = pipeline.runLayer(
                    String.format("Strafe %.0f", MathUtil.inputModulus(rotation.getDegrees(), -180, +180)),
                    input -> input.linearVelocity(translation)
                            .fieldRelativeDisabled()
                            .angularCoefficient(0.3));
            xbox.pov(pov).whileTrue(activateLayer);
        }
    }

    /**
     * Configures the bindings for the operator controller.
     *
     * @param xbox The operator controller
     */
    private void configureOperatorControllerBindings(CommandXboxController xbox) {

        final LauncherControlManual manualLaunchControl = new LauncherControlManual(ManualLaunchMode.Y);

        launcher.setManualModeState(manualLaunchControl);

        final LauncherRunMode DEFAULT_LAUNCH = LauncherRunMode.INTERPOLATION;

        launcher.setMode(DEFAULT_LAUNCH);

        final Trigger cancelButton = xbox.b();

        final Trigger manualButton = xbox.back();
        final Trigger resetButton = xbox.start().debounce(0.01);
        final Trigger interpolationOffsetButton = xbox.x();

        final Trigger intakeFullTrigger = xbox.leftTrigger(0.5);
        final Trigger intakePartialTrigger = xbox.leftTrigger(0.2);

        RobotModeTriggers.disabled()
                .debounce(1)
                .onTrue(launcher.runOnce(launcher::stop).ignoringDisable(true))
                .onTrue(hopper.runOnce(() -> hopper.setMode(HopperRunMode.STOPPED))
                        .ignoringDisable(true))
                .onTrue(intake.runOnce(() -> intake.setMode(IntakeRunMode.UP)).ignoringDisable(true));

        // --- INTAKE CONTROL ---

        // Intake button (hold)
        intakeFullTrigger
                .whileTrue(intake.runEnd(
                                () -> intake.setMode(IntakeRunMode.INTAKING),
                                () -> intake.setMode(
                                        intake.getDefaultMode() == IntakeRunMode.UP
                                                ? IntakeRunMode.POST_INTAKE_TRANSITION
                                                : intake.getDefaultMode()))
                        .withName("Intake"))
                .whileTrue(hopper.runEnd(
                                () -> hopper.setMode(HopperRunMode.IDLE), () -> hopper.setMode(HopperRunMode.STOPPED))
                        .withName("Hopper Intake"));

        // Start to automatically push ball
        intakePartialTrigger.onFalse(Commands.waitSeconds(0.3)
                .andThen(intake.runOnce(() -> intake.setMode(IntakeRunMode.UP)))
                .onlyWhile(() -> intake.getMode() == IntakeRunMode.POST_INTAKE_TRANSITION)
                .withName("Agitate Post Intake"));

        // Dump through intake button (hold)
        xbox.leftBumper()
                .debounce(0.1)
                .whileTrue(intake.runEnd(
                                () -> intake.setMode(IntakeRunMode.OUTTAKING_DUMP),
                                () -> intake.setMode(intake.getDefaultMode()))
                        .withName("Dump Intake"))
                .whileTrue(hopper.runEnd(
                                () -> hopper.setMode(HopperRunMode.REVERSE),
                                () -> hopper.setMode(HopperRunMode.STOPPED))
                        .withName("Dump Hopper"));

        xbox.leftStick().onTrue(Commands.runOnce(() -> intake.setDefaultMode(IntakeRunMode.UP)));

        // Agitate button (hold)
        xbox.leftStick()
                .and(xbox.x())
                .whileTrue(Commands.sequence(
                                intake.runOnce(() -> intake.setMode(IntakeRunMode.AGITATE_1_UP)),
                                Commands.waitSeconds(0.5),
                                intake.runOnce(() -> intake.setMode(IntakeRunMode.AGITATE_2)),
                                Commands.waitSeconds(0.3))
                        .repeatedly()
                        .withName("Agitate")
                        .finallyDo(() -> intake.setMode(intake.getDefaultMode())));

        xbox.leftStick().and(xbox.x().negate()).whileTrue(new StagedAgitateFeed(intake).withName("Staged Agitate"));

        // Deploy intake tap button
        xbox.rightStick()
                .whileTrue(intake.run(() -> intake.setModeNoWheels(IntakeRunMode.INTAKING_NO_WHEELS))
                        .withName("Deploy intake no wheels"))
                .onTrue(Commands.runOnce(() -> intake.setDefaultMode(IntakeRunMode.INTAKING_NO_WHEELS)));

        // Intake shift up button
        xbox.povUp()
                .and(manualButton.negate())
                .and(interpolationOffsetButton.negate())
                .onTrue(Commands.runOnce(() -> intake.shiftSetpoint(Rotation2d.fromDegrees(+1)))
                        .withName("Shift intake up"));

        // Intake shift down button
        xbox.povDown()
                .and(manualButton.negate())
                .and(interpolationOffsetButton.negate())
                .onTrue(Commands.runOnce(() -> intake.shiftSetpoint(Rotation2d.fromDegrees(-1)))
                        .withName("Shift intake down"));

        // Reset intake shift button
        resetButton
                .and(manualButton.negate())
                .and(interpolationOffsetButton.negate())
                .onTrue(Commands.runOnce(intake::unshiftSetpoint).withName("Reset intake shift"));

        // --- OUTTAKE CONTROL ---

        // Spin up then launch button
        xbox.rightTrigger()
                .and(cancelButton.negate())
                .whileTrue(launcher.startEnd(launcher::start, launcher::stop).withName("Spin up for Launch"));

        xbox.rightTrigger()
                .and(launcher::isReadyDebounced)
                .whileTrue(hopper.runEnd(
                                () -> hopper.setMode(HopperRunMode.FIRING), () -> hopper.setMode(HopperRunMode.STOPPED))
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .onlyWhile(launcher::isRunning)
                        .withName("Hopper firing when Ready"));

        // Force launch button
        xbox.rightBumper()
                .and(cancelButton.negate())
                .whileTrue(launcher.run(launcher::start)
                        .alongWith(hopper.run(() -> hopper.setMode(HopperRunMode.FIRING)))
                        .finallyDo(() -> {
                            launcher.stop();
                            hopper.setMode(HopperRunMode.STOPPED);
                        })
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withName("Force Launch"));

        // Interpolation offsets
        xbox.povRight().and(interpolationOffsetButton).onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance()
                .incrementHoodPosition(0.05)));
        xbox.povLeft().and(interpolationOffsetButton).onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance()
                .incrementHoodPosition(-0.05)));
        xbox.povUp().and(interpolationOffsetButton).onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance()
                .incrementWheelRadPerSec(10)));
        xbox.povDown().and(interpolationOffsetButton).onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance()
                .incrementWheelRadPerSec(-10)));
        resetButton.and(interpolationOffsetButton).onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance()
                .resetOffsets()));

        // --- LAUNCH PREP CONTROLS ---

        // Start spin up button
        xbox.y()
                .and(manualButton.negate())
                .whileTrue(launcher.startEnd(launcher::start, launcher::stop).withName("Spin Up"));

        // Cancel spin up button
        cancelButton
                .and(manualButton.negate())
                .onTrue(launcher.runOnce(launcher::stop)
                        .andThen(hopper.runOnce(() -> hopper.setMode(HopperRunMode.STOPPED)))
                        .withName("Cancel Spin Up"));

        // Reverse lifter & outtake button
        xbox.a()
                .and(manualButton.negate())
                .whileTrue(launcher.startEnd(() -> launcher.setDutyCycle(-0.1), launcher::stop)
                        .withName("Reverse Launcher"))
                .whileTrue(hopper.startEnd(
                                () -> hopper.setMode(HopperRunMode.REVERSE),
                                () -> hopper.setMode(HopperRunMode.STOPPED))
                        .withName("Reverse Hopper"));

        // --- MANUAL LAUNCH MODE CONTROLS ---

        final Trigger anyManualModeLetterButton = xbox.a().or(xbox.b()).or(xbox.y());

        // Manual mode is turned on when preset is chosen
        manualButton
                .and(anyManualModeLetterButton)
                .onTrue(Commands.runOnce(() -> launcher.setMode(LauncherRunMode.MANUAL))
                        .ignoringDisable(true)
                        .withName("Manual Launch Mode"));

        // Manual mode is turned off on double tap
        manualButton
                .multiPress(2, 0.3)
                .and(anyManualModeLetterButton.negate())
                .onFalse(Commands.runOnce(() -> launcher.setMode(DEFAULT_LAUNCH))
                        .ignoringDisable(true)
                        .withName("Default Launch Mode"));

        manualButton
                .and(xbox.x().multiPress(2, 0.3))
                .onTrue(Commands.runOnce(() -> launcher.setMode(LauncherRunMode.DASHBOARD_TUNING))
                        .ignoringDisable(true)
                        .withName("Dashboard Tuning Launch Mode"));

        // Manual mode preset buttons
        xbox.y().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.Y));
        xbox.a().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.A));
        xbox.b().and(manualButton).onTrue(manualLaunchControl.setModeCommand(ManualLaunchMode.B));

        // Manual mode preset adjustment buttons
        xbox.povRight().and(manualButton).onTrue(manualLaunchControl.incrementHoodCommand(+0.05));
        xbox.povLeft().and(manualButton).onTrue(manualLaunchControl.incrementHoodCommand(-0.05));
        xbox.povUp().and(manualButton).onTrue(manualLaunchControl.incrementVelocityCommand(+10));
        xbox.povDown().and(manualButton).onTrue(manualLaunchControl.incrementVelocityCommand(-10));
        resetButton.and(manualButton).onTrue(manualLaunchControl.resetCommand());

        // --- HANG/MANUAL CONTROL ---

        // Hang up/down axis
        xbox.getRightY();
    }

    private Command rumbleController(CommandXboxController controller, double rumbleIntensity, RumbleType type) {
        return Commands.startEnd(() -> controller.setRumble(type, rumbleIntensity), () -> controller.setRumble(type, 0))
                .withName("Rumble Controller " + controller.getHID().getPort());
    }

    private Command rumbleControllers(double rumbleIntensity, RumbleType type) {
        if (operatorController == null) {
            return rumbleController(driverController, rumbleIntensity, type);
        }
        return Commands.parallel(
                        rumbleController(driverController, rumbleIntensity, type),
                        rumbleController(operatorController, rumbleIntensity, type))
                .withName("Rumble Both Controllers");
    }

    /** Configures triggers for alerts and robot mode changes. */
    private void configureAlertTriggers() {
        // new Trigger(() -> HubShiftUtil.getOfficialShiftInfo().active())
        //         .onChange(rumbleControllers(1.0, RumbleType.kRightRumble).withTimeout(0.75));

        new Trigger(launcher::isRunning)
                .whileTrue(rumbleControllers(0.3, RumbleType.kLeftRumble).withName("Launcher Running Rumble"));

        new Trigger(launcher::isRunning)
                .and(() -> hopper.getCurrentRunMode() == HopperRunMode.FIRING)
                .onTrue(rumbleControllers(0.5, RumbleType.kRightRumble)
                        .withTimeout(0.5)
                        .withName("Launcher Ready Rumble"));

        Trigger isMatch = new Trigger(() -> DriverStation.getMatchTime() != -1);

        if (Constants.isDemoMode()) {
            Elastic.selectTab("Demo");
        }

        RobotModeTriggers.teleop().and(isMatch).onTrue(Commands.runOnce(() -> Elastic.selectTab("Teleoperated")));

        RobotModeTriggers.autonomous().and(isMatch).onTrue(Commands.runOnce(() -> Elastic.selectTab("Autonomous")));

        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled()
                .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    }

    /** Configures the LED commands. */
    private void configureLEDs() {
        LoggedDashboardChooser<BlinkinLEDPattern> ledFallbackPatternChooser =
                new LoggedDashboardChooser<>("LED Pattern Chooser", new SendableChooser<BlinkinLEDPattern>());

        final BlinkinLEDPattern defaultPattern = BlinkinLEDPattern.GOLD;

        SmartDashboard.putData("LED Default Pattern Chooser", ledFallbackPatternChooser.getSendableChooser());

        ledFallbackPatternChooser.addDefaultOption(String.format("Default (%s)", defaultPattern), defaultPattern);

        for (BlinkinLEDPattern pattern : BlinkinLEDPattern.values()) {
            ledFallbackPatternChooser.addOption(pattern.toString(), pattern);
        }

        new Trigger(() -> HubShiftUtil.getOfficialShiftInfo().active())
                .onTrue(leds.runColor(BlinkinLEDPattern.GREEN)
                        .withName("Hub Shift Active LED")
                        .withTimeout(0.25))
                .onFalse(leds.runColor(BlinkinLEDPattern.WHITE)
                        .withName("Hub Shift Inactive LED")
                        .withTimeout(0.25));

        leds.setDefaultCommand(leds.runColor(() -> {
                    if (DriverStation.isAutonomous()) {
                        return BlinkinLEDPattern.FIRE_LARGE;
                    }

                    LaunchingParameters launchParams =
                            LaunchCalculator.getInstance().getParameters(drive.getRobotPose(), drive.getRobotSpeeds());

                    ShiftInfo shift = HubShiftUtil.getOfficialShiftInfo();
                    if (shift.active() && launchParams.isValid() && !launchParams.passing()) {
                        return BlinkinLEDPattern.BLUE_GREEN;
                    }

                    if (shift.currentShift() == ShiftEnum.TRANSITION) {
                        if (HubShiftUtil.isFirstActiveAlliance()) {
                            return BlinkinLEDPattern.GREEN;
                        } else {
                            return BlinkinLEDPattern.WHITE;
                        }
                    }

                    return ledFallbackPatternChooser.get();
                })
                .withName("LED Default Supplied Color"));
    }

    /** Configures the LED commands. */
    private void configureDemoLEDs() {
        LoggedDashboardChooser<BlinkinLEDPattern> ledFallbackPatternChooser =
                new LoggedDashboardChooser<>("LED Pattern Chooser", new SendableChooser<BlinkinLEDPattern>());

        final BlinkinLEDPattern defaultPattern = BlinkinLEDPattern.GOLD;

        SmartDashboard.putData("LED Default Pattern Chooser", ledFallbackPatternChooser.getSendableChooser());

        ledFallbackPatternChooser.addDefaultOption(String.format("Default (%s)", defaultPattern), defaultPattern);

        for (BlinkinLEDPattern pattern : BlinkinLEDPattern.values()) {
            ledFallbackPatternChooser.addOption(pattern.toString(), pattern);
        }

        leds.setDefaultCommand(leds.runColor(() -> {
                    if (DriverStation.isAutonomous()) {
                        return BlinkinLEDPattern.RED;
                    }

                    if (launcher.isRunning()) {
                        if (hopper.getCurrentRunMode() == HopperRunMode.FIRING) {
                            return BlinkinLEDPattern.FIRE_LARGE;
                        } else {
                            return BlinkinLEDPattern.ORANGE;
                        }
                    }

                    if (intake.getMode() == IntakeRunMode.INTAKING) {
                        return BlinkinLEDPattern.GREEN;
                    }

                    return ledFallbackPatternChooser.get();
                })
                .withName("LED Default Supplied Color"));
    }

    /** Make commands accessible to PathPlanner autos. */
    private void registerNamedCommands() {
        LinkedHashMap<String, Command> namedCommands = new LinkedHashMap<String, Command>();

        namedCommands.put("LEDS", leds.runColor(BlinkinLEDPattern.RED));

        // Hopper commands
        namedCommands.put("StopHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.STOPPED)));
        namedCommands.put("IdleHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.IDLE)));
        namedCommands.put("FireHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.FIRING)));
        namedCommands.put("ReverseHopper", hopper.runOnce(() -> hopper.setMode(HopperRunMode.REVERSE)));

        // Intake commands
        namedCommands.put("ExtendSlapdown", intake.runOnce(() -> intake.setModeNoWheels(IntakeRunMode.INTAKING)));
        namedCommands.put("RetractSlapdown", intake.runOnce(() -> intake.setModeNoWheels(IntakeRunMode.UP)));
        namedCommands.put("StartIntake", intake.runOnce(() -> intake.setMode(IntakeRunMode.INTAKING)));
        namedCommands.put("StopIntake", intake.runOnce(() -> intake.setMode(IntakeRunMode.UP)));

        // Launcher commands
        namedCommands.put("PrimeToLaunch", LaunchCommands.primeToLaunch(drive, launcher));
        namedCommands.put("LaunchInPlace", LaunchCommands.launchInPlace(drive, vision, launcher, hopper, 0));
        namedCommands.put(
                "LaunchInPlaceAgitate",
                LaunchCommands.launchInPlaceAgitate(drive, vision, launcher, hopper, intake, 2.5));

        // Hang commands
        namedCommands.put("HangUp", Commands.none());
        namedCommands.put("HangDown", Commands.none());

        namedCommands.put(
                "WaitVariable",
                Commands.defer(
                        () -> Commands.waitSeconds(MathUtil.clamp(DriverDashboard.getDelaySeconds(), 0, 10)),
                        Set.of()));

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

        var chooser = AutoBuilder.buildAutoChooser();

        if (Constants.DEVELOPMENT_MODE) {
            chooser.addOption(
                    "[Characterization] Drive Feed Forward",
                    DriveCharacterizationCommands.feedforwardCharacterization(drive));
            chooser.addOption(
                    "[Characterization] Drive Wheel Radius",
                    DriveCharacterizationCommands.wheelRadiusCharacterization(drive));

            chooser.addOption(
                    "[SysId] Drive Quasistatic Forward", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            chooser.addOption(
                    "[SysId] Drive Quasistatic Reverse", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            chooser.addOption("[SysId] Drive Dynamic Forward", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            chooser.addOption("[SysId] Drive Dynamic Reverse", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        } else {
            chooser.addOption("Nothing", Commands.none());
        }

        return chooser;
    }

    private void setupInitPose() {
        Pose2d startingPose =
                new Pose2d(FieldConstants.LinesVertical.starting, FieldConstants.fieldWidth / 2.0, Rotation2d.kZero);

        Pose2d staringPoseFallback = new Pose2d(
                new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth).div(2), Rotation2d.kCW_90deg);

        drive.resetPose(staringPoseFallback);

        CommandScheduler.getInstance()
                .schedule(Commands.waitUntil(() -> DriverStation.getAlliance().isPresent())
                        .andThen(Commands.runOnce(() -> drive.resetPose(FieldFlipUtil.apply(startingPose))))
                        .withTimeout(3)
                        .onlyWhile(() -> drive.getRobotPose() == staringPoseFallback)
                        .ignoringDisable(true)
                        .withName("Init Pose"));
    }
}
