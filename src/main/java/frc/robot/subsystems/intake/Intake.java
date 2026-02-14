package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import org.littletonrobotics.junction.Logger;

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

  private Mechanism2d mech = new Mechanism2d(3, 3);
  private MechanismRoot2d root = mech.getRoot("wheel", 1.5, 1.5);
  private MechanismLigament2d slapdownArm =
      root.append(
          new MechanismLigament2d(
              "slapdownArm",
              0.5,
              IntakeConstants.SLAPDOWN_UP_SETPOINT.getDegrees(),
              10,
              new Color8Bit(Color.kOrange)));
  private MechanismLigament2d wheelArm1 =
      slapdownArm.append(
          new MechanismLigament2d("wheelArm1", 0.09, 90, 10, new Color8Bit(Color.kRed)));
  private MechanismLigament2d wheelArm2 =
      slapdownArm.append(
          new MechanismLigament2d("wheelArm2", 0.09, 180, 10, new Color8Bit(Color.kRed)));

  public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
    this.wheelIO = wheelIO;
    this.slapdownIO = slapdownIO;

    setSlapdownPID(
        IntakeConstants.SLAPDOWN_PID.kP(),
        IntakeConstants.SLAPDOWN_PID.kI(),
        IntakeConstants.SLAPDOWN_PID.kD());

    wheelInputs = new IntakeWheelIOInputsAutoLogged();
    slapdownInputs = new SlapdownIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    wheelIO.updateInputs(wheelInputs);
    slapdownIO.updateInputs(slapdownInputs);

    Logger.processInputs(getName() + "/Wheel", wheelInputs);
    Logger.processInputs(getName() + "/Slapdown", slapdownInputs);

    slapdownArm.setAngle(Units.radiansToDegrees(slapdownInputs.positionRad));
    wheelArm1.setAngle(Units.radiansToDegrees(wheelInputs.positionRad));
    wheelArm2.setAngle(Units.radiansToDegrees(wheelInputs.positionRad + Math.PI));

    SmartDashboard.putNumber("Wheel Speed", wheelIO.getSpeed());
    SmartDashboard.putData("intakeMech", mech);

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

  public void setSlapdownPID(double kP, double kI, double kD) {
    slapdownIO.setPID(kP, kI, kD);
  }

  public void setSlapdownSetpoint(Rotation2d setPoint) {
    slapdownIO.setSetpoint(setPoint);
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
