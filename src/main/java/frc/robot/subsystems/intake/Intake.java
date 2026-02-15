package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheelIO;
  private final SlapdownIO slapdownIO;

  private final Alert wheelMotorDisconnectedAlert =
      new Alert("Hardware error detected on intake wheel.", AlertType.kError);
  private final Alert slapdownMotorDisconnectedAlert =
      new Alert("Hardware error detected on slapdown.", AlertType.kError);
  private final Alert encodersMisalignedAlert =
      new Alert("Absolute & relative encoders on slapdown misaligned.", AlertType.kWarning);

  private final TunablePID slapdownPidConfig =
      new TunablePID(getName() + "/Slapdown/Pid", IntakeConstants.SLAPDOWN_PID);

  private IntakeWheelIOInputsAutoLogged wheelInputs;
  private SlapdownIOInputsAutoLogged slapdownInputs;

  private final IntakeVisualizer visualizer;

  public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
    this.wheelIO = wheelIO;
    this.slapdownIO = slapdownIO;

    slapdownIO.setPID(IntakeConstants.SLAPDOWN_PID);

    wheelInputs = new IntakeWheelIOInputsAutoLogged();
    slapdownInputs = new SlapdownIOInputsAutoLogged();

    slapdownIO.setSetpoint(IntakeConstants.SLAPDOWN_UP_SETPOINT);

    visualizer =
        new IntakeVisualizer(
            getName(), () -> slapdownInputs.positionRad, () -> wheelInputs.positionRad);
  }

  @Override
  public void periodic() {
    wheelIO.updateInputs(wheelInputs);
    slapdownIO.updateInputs(slapdownInputs);

    Logger.processInputs(getName() + "/Wheel", wheelInputs);
    Logger.processInputs(getName() + "/Slapdown", slapdownInputs);

    slapdownPidConfig.ifChanged(hashCode(), () -> slapdownIO.setPID(slapdownPidConfig.get()));

    wheelMotorDisconnectedAlert.set(!wheelInputs.motorConnected);
    slapdownMotorDisconnectedAlert.set(!slapdownInputs.motorConnected);
    encodersMisalignedAlert.set(!slapdownInputs.encodersAligned);
  }

  // wheel

  public void setWheelSpeed(double speed) {
    wheelIO.setSpeed(speed);
  }

  public void stopWheels() {
    wheelIO.stop();
  }

  // slapdown

  public void setSlapdownSetpoint(Rotation2d setPoint) {
    slapdownIO.setSetpoint(setPoint);
  }

  public void setSavedUpSetpoint(Rotation2d setPoint) {
    slapdownIO.setSavedUpSetpoint(setPoint);
  }

  public void setSavedDownSetpoint(Rotation2d setPoint) {
    slapdownIO.setSavedDownSetpoint(setPoint);
  }

  public Rotation2d getSavedUpSetpoint() {
    return slapdownIO.getSavedUpSetpoint();
  }

  public Rotation2d getSavedDownSetpoint() {
    return slapdownIO.getSavedDownSetpoint();
  }

  public static Intake create(RobotType robotType) {

    switch (robotType) {
      case METALBOT_2:
        return new Intake(new IntakeWheelIO() {}, new SlapdownIO() {});

      case PRESEASON_2026:
        return new Intake(new IntakeWheelIO() {}, new SlapdownIO() {});

      case CHASSIS_CANNON:
      case WOOD_BOT_2026:
      case REEFSCAPE_2025:
        return new Intake(new IntakeWheelIO() {}, new SlapdownIO() {});

      case SIM_BOT:
        return new Intake(new IntakeWheelIOSim(), new SlapdownIOSim());

      default:
        return new Intake(new IntakeWheelIO() {}, new SlapdownIO() {});
    }
  }
}
