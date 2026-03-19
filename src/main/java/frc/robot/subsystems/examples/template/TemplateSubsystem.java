package frc.robot.subsystems.examples.template;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The subsystem that the person will actually use for the Template. */
public class TemplateSubsystem extends SubsystemBase {
  private final TemplateIO io;
  private final TemplateIOInputsAutoLogged inputs = new TemplateIOInputsAutoLogged();

  /** Creates a new Template. */
  public TemplateSubsystem(TemplateIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Template", inputs);
  }

  /** Run the motor at a given speed */
  public void start() {
    io.setSpeed(TemplateConstants.SPEED);
  }

  /** Stop the motor */
  public void stop() {
    io.setSpeed(0.0);
  }

  public double getRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
