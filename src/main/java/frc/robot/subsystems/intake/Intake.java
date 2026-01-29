package frc.robot.subsystems.intake;

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
}
