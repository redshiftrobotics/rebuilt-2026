package frc.robot.subsystems.launcher;

/** Simulation implementation of the TemplateIO. */
public class HoodActuatorIOSim implements HoodActuatorIO {

  public HoodActuatorIOSim() {}

  @Override
  public void updateInputs(HoodActuatorIOInputs inputs) {
    inputs.position = 0;
  }
}
