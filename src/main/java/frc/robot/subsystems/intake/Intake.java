package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.intake.IntakeConstants.SlapdownConstants;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.Logger;

/**
 * The Intake subsystem controls both the intake wheels and the slapdown mechanism.
 *
 * <p>The intake wheels are responsible for intaking and outtaking game pieces, while the slapdown
 * mechanism adjusts the intake's angle for different scoring positions.
 *
 * <p>This subsystem supports three robot configurations: the competition robot (REBUILT_2026),
 * simulation (SIM_BOT), and a default fallback for unknown robots.
 */
public class Intake extends SubsystemBase {
    private final IntakeWheelIO wheelIO;
    private final SlapdownIO slapdownIO;

    private IntakeWheelIOInputsAutoLogged wheelInputs = new IntakeWheelIOInputsAutoLogged();
    private SlapdownIOInputsAutoLogged slapdownInputs = new SlapdownIOInputsAutoLogged();

    private IntakeRunMode currentMode = IntakeRunMode.UP;

    private final Alert wheelMotorDisconnectedAlert =
            new Alert("Hardware error detected on intake wheel.", AlertType.kError);
    private final Alert slapdownMotorDisconnectedAlert =
            new Alert("Hardware error detected on slapdown.", AlertType.kError);
    private final Alert encodersMisalignedAlert =
            new Alert("Absolute & relative encoders on slapdown misaligned.", AlertType.kWarning);

    private final TunablePID slapdownPidConfig = new TunablePID(getName() + "/SlapdownPID", SlapdownConstants.PID);

    private final IntakeVisualizer visualizer;
    private final IntakeVisualizer absoluteVisualizer;
    private final IntakeVisualizer setpointVisualizer;

    public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
        this.wheelIO = wheelIO;
        this.slapdownIO = slapdownIO;

        slapdownIO.setPID(SlapdownConstants.PID);

        visualizer = new IntakeVisualizer(getName() + "/Visuization/Measured", Color.kRed);
        absoluteVisualizer = new IntakeVisualizer(getName() + "/Visuization/AbsoluteMeasured", Color.kOrange);
        setpointVisualizer = new IntakeVisualizer(getName() + "/Visuization/Setpoint", Color.kBlue);

        setMode(IntakeRunMode.UP);
    }

    @Override
    public void periodic() {
        wheelIO.updateInputs(wheelInputs);
        slapdownIO.updateInputs(slapdownInputs);

        Logger.processInputs(getName() + "/Wheel", wheelInputs);
        Logger.processInputs(getName() + "/Slapdown", slapdownInputs);

        slapdownPidConfig.ifChanged(hashCode(), slapdownIO::setPID);

        Logger.recordOutput(getName() + "/mode", currentMode.toString());

        visualizer.update(slapdownInputs.positionRad, wheelInputs.positionRad);
        absoluteVisualizer.update(slapdownInputs.absolutePositionRad, wheelInputs.positionRad);
        wheelMotorDisconnectedAlert.set(!wheelInputs.motorConnected);
        slapdownMotorDisconnectedAlert.set(!slapdownInputs.motorConnected);
        encodersMisalignedAlert.set(!slapdownInputs.encodersAligned);
    }

    public void shiftSetpoint(Rotation2d shift) {
        currentMode.shiftSetpoint(shift);
        setMode(currentMode);
    }

    public void unshiftSetpoint() {
        currentMode.resetShift();
        setMode(currentMode);
    }

    public void setMode(IntakeRunMode mode) {
        currentMode = mode;
        wheelIO.setSpeed(mode.intakeDutyCycle);
        slapdownIO.setSetpoint(currentMode.getSetpoint());
        setpointVisualizer.update(
                currentMode.getSetpoint().getRadians(),
                currentMode.intakeDutyCycle
                        * IntakeConstants.IntakeWheelConstants.MOTOR.freeSpeedRadPerSec
                        * Units.millisecondsToSeconds(System.currentTimeMillis()));
    }

    public void setModeNoWheels(IntakeRunMode mode) {
        currentMode = mode;
        wheelIO.stop();
        slapdownIO.setSetpoint(currentMode.getSetpoint());
        setpointVisualizer.update(currentMode.getSetpoint().getRadians(), 0);
    }

    /**
     * Creates an Intake subsystem for the specified robot type.
     *
     * @param robotType The type of robot to create the intake for
     * @return The configured Intake subsystem instance
     */
    public static Intake create(RobotType robotType) {

        switch (robotType) {
            case REBUILT_2026:
                return new Intake(new IntakeWheelIOSparkMax(), new SlapdownIOSparkMax());

            case METALBOT_2:
            case SIM_BOT:
                return new Intake(new IntakeWheelIOSim(), new SlapdownIOSim());

            default:
                return new Intake(new IntakeWheelIO() {}, new SlapdownIO() {});
        }
    }
}
