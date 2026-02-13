package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheelIO;
  private final SlapdownIO slapdownIO;

  private IntakeWheelIOInputsAutoLogged wheelInputs;
  private SlapdownIOInputsAutoLogged slapdownInputs;

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

    SmartDashboard.putNumber("Wheel Speed", wheelIO.getSpeed());
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
