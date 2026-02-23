package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.intake.IntakeConstants.SlapdownConstants;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheelIO;
  private final SlapdownIO slapdownIO;

  private IntakeWheelIOInputsAutoLogged wheelInputs = new IntakeWheelIOInputsAutoLogged();
  private SlapdownIOInputsAutoLogged slapdownInputs = new SlapdownIOInputsAutoLogged();

  private Rotation2d slapdownUpPosition;
  private Rotation2d slapdownDownPosition;

  private Rotation2d setpointPosition = Rotation2d.kZero;
  private double setpointWheelSpeed;

  private final Alert wheelMotorDisconnectedAlert =
      new Alert("Hardware error detected on intake wheel.", AlertType.kError);
  private final Alert slapdownMotorDisconnectedAlert =
      new Alert("Hardware error detected on slapdown.", AlertType.kError);
  private final Alert encodersMisalignedAlert =
      new Alert("Absolute & relative encoders on slapdown misaligned.", AlertType.kWarning);

  private final TunablePID slapdownPidConfig =
      new TunablePID(getName() + "/SlapdownPID", SlapdownConstants.PID);

  private final IntakeVisualizer visualizer;
  private final IntakeVisualizer absoluteVisualizer;
  private final IntakeVisualizer setpointVisualizer;

  public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
    this.wheelIO = wheelIO;
    this.slapdownIO = slapdownIO;

    slapdownIO.setPID(SlapdownConstants.PID);

    visualizer = new IntakeVisualizer(getName() + "/Visuization/Measured", Color.kRed);
    absoluteVisualizer =
        new IntakeVisualizer(getName() + "/Visuization/AbsoluteMeasured", Color.kOrange);
    setpointVisualizer = new IntakeVisualizer(getName() + "/Visuization/Setpoint", Color.kBlue);

    slapdownUpPosition = SlapdownConstants.UP_SETPOINT;
    slapdownDownPosition = SlapdownConstants.DOWN_SETPOINT;

    setSlapdownSetpoint(slapdownUpPosition);
    stopWheels();

    SmartDashboard.putData(
        "Intake State",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty(
                "Slapdown Setpoint (deg)", () -> setpointPosition.getDegrees(), null);
            builder.addDoubleProperty(
                "Slapdown Position (deg)",
                () -> Units.radiansToDegrees(slapdownInputs.positionRad),
                null);
            builder.addDoubleProperty(
                "Slapdown Absolute Position (deg)",
                () -> Units.radiansToDegrees(slapdownInputs.absolutePositionRad),
                null);
            builder.addDoubleProperty("Wheel Dutycycle", () -> setpointWheelSpeed, null);
          }
        });
  }

  @Override
  public void periodic() {
    wheelIO.updateInputs(wheelInputs);
    slapdownIO.updateInputs(slapdownInputs);

    Logger.processInputs(getName() + "/Wheel", wheelInputs);
    Logger.processInputs(getName() + "/Slapdown", slapdownInputs);

    slapdownPidConfig.ifChanged(hashCode(), slapdownIO::setPID);

    visualizer.update(slapdownInputs.positionRad, wheelInputs.positionRad);
    absoluteVisualizer.update(slapdownInputs.absolutePositionRad, wheelInputs.positionRad);
    setpointVisualizer.update(
        setpointPosition.getRadians(), -setpointWheelSpeed * System.currentTimeMillis() * 0.001);

    wheelMotorDisconnectedAlert.set(!wheelInputs.motorConnected);
    slapdownMotorDisconnectedAlert.set(!slapdownInputs.motorConnected);
    encodersMisalignedAlert.set(!slapdownInputs.encodersAligned);
  }

  // wheel

  public void setWheelSpeed(double speed) {
    wheelIO.setSpeed(speed);
    this.setpointWheelSpeed = speed;
  }

  public void stopWheels() {
    wheelIO.stop();
    this.setpointWheelSpeed = 0;
  }

  // slapdown

  public void setSlapdownSetpoint(Rotation2d setpoint) {
    slapdownIO.setSetpoint(setpoint);
    this.setpointPosition = setpoint;
  }

  public void setSavedUpSetpoint(Rotation2d setPoint) {
    setSlapdownSetpoint(setPoint);
    slapdownUpPosition = setPoint;
  }

  public void setSavedDownSetpoint(Rotation2d setPoint) {
    setSlapdownSetpoint(setPoint);
    slapdownDownPosition = setPoint;
  }

  public Rotation2d getSavedUpSetpoint() {
    return slapdownUpPosition;
  }

  public Rotation2d getSavedDownSetpoint() {
    return slapdownDownPosition;
  }

  public static Intake create(RobotType robotType) {

    switch (robotType) {
      case REBUILT_2026:
        return new Intake(new IntakeWheelIOSparkMax(), new SlapdownIOSparkMax());

      case SIM_BOT:
        return new Intake(new IntakeWheelIOSim(), new SlapdownIOSim());

      default:
        return new Intake(new IntakeWheelIO() {}, new SlapdownIO() {});
    }
  }
}
