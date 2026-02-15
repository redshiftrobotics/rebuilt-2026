package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.utility.records.PIDConstants;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheelIO;
  private final SlapdownIO slapdownIO;

  private final Alert wheelMotorDisconnectedAlert =
      new Alert("Sticky fault detected on intake wheel.", AlertType.kError);
  private final Alert slapdownMotorDisconnectedAlert =
      new Alert("Sticky fault detected on slapdown.", AlertType.kError);

  private final TunablePID slapdownPidConfig =
      new TunablePID(getName() + "/Slapdown/Pid", IntakeConstants.SLAPDOWN_PID);

  private IntakeWheelIOInputsAutoLogged wheelInputs;
  private SlapdownIOInputsAutoLogged slapdownInputs;

  private LoggedMechanism2d visualizerMechanism = new LoggedMechanism2d(3, 1.5);
  private LoggedMechanismRoot2d visualizerRoot = visualizerMechanism.getRoot("wheel", 1.75, .25);
  private LoggedMechanismLigament2d visualizerSlapdownArm =
      visualizerRoot.append(
          new LoggedMechanismLigament2d(
              "slapdownArm",
              0.5,
              IntakeConstants.SLAPDOWN_UP_SETPOINT.getDegrees(),
              10,
              new Color8Bit(Color.kOrange)));
  private LoggedMechanismLigament2d wheelArm1 =
      visualizerSlapdownArm.append(
          new LoggedMechanismLigament2d("wheelArm1", 0.09, 90, 10, new Color8Bit(Color.kRed)));
  private LoggedMechanismLigament2d wheelArm2 =
      visualizerSlapdownArm.append(
          new LoggedMechanismLigament2d("wheelArm2", 0.09, 180, 10, new Color8Bit(Color.kRed)));

  public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
    this.wheelIO = wheelIO;
    this.slapdownIO = slapdownIO;

    slapdownIO.setPID(IntakeConstants.SLAPDOWN_PID);

    wheelInputs = new IntakeWheelIOInputsAutoLogged();
    slapdownInputs = new SlapdownIOInputsAutoLogged();

    slapdownIO.setSetpoint(IntakeConstants.SLAPDOWN_UP_SETPOINT);
  }

  @Override
  public void periodic() {
    wheelIO.updateInputs(wheelInputs);
    slapdownIO.updateInputs(slapdownInputs);

    Logger.processInputs(getName() + "/Wheel", wheelInputs);
    Logger.processInputs(getName() + "/Slapdown", slapdownInputs);

    slapdownPidConfig.ifChanged(hashCode(), () -> slapdownIO.setPID(slapdownPidConfig.get()));

    visualizerSlapdownArm.setAngle(Units.radiansToDegrees(slapdownInputs.positionRad));
    wheelArm1.setAngle(Units.radiansToDegrees(wheelInputs.positionRad));
    wheelArm2.setAngle(Units.radiansToDegrees(wheelInputs.positionRad + Math.PI));

    Logger.recordOutput(getName() + "/Visualization", visualizerMechanism);

    wheelMotorDisconnectedAlert.set(!wheelInputs.motorConnected);
    slapdownMotorDisconnectedAlert.set(!slapdownInputs.motorConnected);
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
