package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeWheelIO.IntakeWheelIOInputsAutoLogged;
import frc.robot.subsystems.intake.SlapdownIO.SlapdownIOInputsAutoLogged;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheelIO;
  private final SlapdownIO SlapdownIO;

  private IntakeWheelIOInputsAutoLogged wheelInputs;
  private SlapdownIOInputsAutoLogged slapdownInputs;

  public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO) {
    this.wheelIO = wheelIO;
    this.SlapdownIO = slapdownIO;
  }

  @Override
  public void periodic() {
    wheelIO.updateInputs(wheelInputs);
    SlapdownIO.updateInputs(slapdownInputs);
  }

  // wheel

  public void wheelSet(double speed) {
    wheelIO.set(speed);
  }

  public void wheelStop() {
    wheelIO.stop();
  }

  // slapdown

  public void setSlapdownPID(double kp, double ki, double kd) {
    SlapdownIO.setPID(kp, ki, kd);
  }

  public void slapdownSet(double speed) {
    SlapdownIO.setSpeed(speed);
  }

  public void setSlapdownIdleMode(IdleMode idleMode) {
    SlapdownIO.setMotorIdleMode(idleMode);
  }

  public void splapdownSetPoint(Rotation2d setPoint) {
    SlapdownIO.setSetpoint(setPoint);
  }

  public boolean slapdownIsAtSetpoint(){
    return SlapdownIO.slapdownIsAtSetpoint();
  }

  public void slapdownStop() {
    SlapdownIO.stopMotor();
  }
}
