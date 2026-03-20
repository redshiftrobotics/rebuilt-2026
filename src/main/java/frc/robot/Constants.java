package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /** The period, in seconds, of the main robot loop */
    public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

    public static final RobotType PRIMARY_ROBOT_TYPE = RobotType.REBUILT_2026;
    private static RobotType robotType;

    /**
     * If true, enables several development features, such as
     *
     * <ul>
     *   <li>TunableNumber editing in AdvantageScope
     *   <li>Verbose logging in AdvantageScope
     *   <li>Enables SysId, Diagnostic, Test Plans, and other debug autos
     */
    public static final boolean DEVELOPMENT_MODE = true;

    /**
     * If true, robot is considered to be on the playing field. Vision will look for field tags, and
     * auto alignment should become active.
     */
    private static final boolean IS_ON_PLAYING_FIELD = true;

    /** If true, enables demo mode features throughout the codebase. */
    private static final boolean DEMO_MODE = false;

    /** Talon noises */
    public static final boolean TALON_BEEP_ON_CONFIG = false;

    /** Talon noises */
    public static final boolean TALON_BEEP_ON_BOOT = false;

    /** If true, logger will attempt to log to a USB stick ("/U/logs") */
    public static final boolean AKIT_LOGGER_LOG_TO_USB = false;

    public static RobotType getRobot() {
        if (robotType == null) {
            robotType = determineRobotType();
            if (robotType == null) {
                wrongRobotTypeFailedDetermination.set(true);
                robotType = PRIMARY_ROBOT_TYPE;
            }
        }
        if (RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
            wrongRobotTypeAlertReal.set(true);
            robotType = PRIMARY_ROBOT_TYPE;
        }
        if (RobotBase.isSimulation() && robotType != RobotType.SIM_BOT) {
            wrongRobotTypeAlertReal.set(true);
            robotType = RobotType.SIM_BOT;
        }
        return robotType;
    }

    public static Mode getMode() {
        return switch (getRobot()) {
            case SIM_BOT -> Mode.SIM;
            default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        };
    }

    public static boolean isOnPlayingField() {
        return DriverStation.isFMSAttached() || IS_ON_PLAYING_FIELD;
    }

    @SuppressWarnings("unused")
    public static boolean isDemoMode() {
        return DEMO_MODE && !DriverStation.isFMSAttached();
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        REBUILT_2026,
        METALBOT_2,
        PRESEASON_2026,
        REEFSCAPE_2025,
        WOOD_BOT_2026,
        CHASSIS_CANNON,
        SIM_BOT,
    }

    private static RobotType determineRobotType() {
        if (RobotBase.isReal()) {
            unrecognizedRobotType.set(false);
            switch (RobotController.getSerialNumber()) {
                case "023AC95B":
                    return RobotType.PRESEASON_2026;
                case "032D2143":
                    return RobotType.CHASSIS_CANNON;
                case "02384981":
                    return RobotType.REEFSCAPE_2025;
                case "032D216B":
                    return RobotType.WOOD_BOT_2026;
                case "03238024":
                    return RobotType.METALBOT_2;
                case "0240C1FA":
                case "0240570E":
                    return RobotType.REBUILT_2026;
                default:
                    unrecognizedRobotType.set(true);
                    return PRIMARY_ROBOT_TYPE;
            }
        } else if (RobotBase.isSimulation()) {
            return RobotType.SIM_BOT;
        }
        return null;
    }

    private static final Alert wrongRobotTypeAlertReal = new Alert(
            String.format("Invalid robot selected, using %s robot as default.", PRIMARY_ROBOT_TYPE.toString()),
            Alert.AlertType.kInfo);

    private static final Alert wrongRobotTypeFailedDetermination = new Alert(
            String.format(
                    "Failed to determine robot from RoboRio serial number, using %s robot as default.",
                    PRIMARY_ROBOT_TYPE.toString()),
            AlertType.kInfo);

    private static final Alert notOnField =
            new Alert("Robot is not on playing field according to Constants.java", AlertType.kInfo);

    private static final Alert demoMode =
            new Alert("Robot is in demo mode according to Constants.java", AlertType.kInfo);

    private static final Alert unrecognizedRobotType =
            new Alert("RoboRIO serial number unrecognized!", AlertType.kWarning);

    static {
        notOnField.set(!isOnPlayingField());
        demoMode.set(isDemoMode());
    }
}
