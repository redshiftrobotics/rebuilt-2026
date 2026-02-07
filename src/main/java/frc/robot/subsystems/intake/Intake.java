package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeWheelIO.IntakeWheelIOInputsAutoLogged;
import frc.robot.subsystems.intake.SlapdownIO.SlapdownIOInputsAutoLogged;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheelIO;
  private final SlapdownIO slapdownIO;

  private IntakeWheelIOInputsAutoLogged wheelInputs;
  private SlapdownIOInputsAutoLogged slapdownInputs;

  public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
    this.wheelIO = wheelIO;
    this.slapdownIO = slapdownIO;

    setSlapdownPID(IntakeConstants.SLAPDOWN_PID.kP(), IntakeConstants.SLAPDOWN_PID.kI(), IntakeConstants.SLAPDOWN_PID.kD());

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

  public void wheelSet(double speed) {
    wheelIO.set(speed);
  }

  public void wheelStop() {
    wheelIO.stop();
  }

  // slapdown

  public void setSlapdownPID(double kP, double kI, double kD) {
    slapdownIO.setPID(kP, kI, kD);
  }

  public void slapdownSet(double speed) {
    slapdownIO.setSpeed(speed);
  }

  public void setSlapdownIdleMode(IdleMode idleMode) {
    slapdownIO.setMotorIdleMode(idleMode);
  }

  public void slapdownSetPoint(Rotation2d setPoint) {
    slapdownIO.setSetpoint(setPoint);
  }

  public boolean slapdownIsAtSetpoint() {
    return slapdownIO.slapdownIsAtSetpoint();
  }

  public void slapdownStop() {
    slapdownIO.stopMotor();
  }
}
