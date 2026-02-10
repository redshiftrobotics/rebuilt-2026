package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.Servo;

/** Hardware implementation of the TemplateIO. */
public class HoodActuatorIOHardware implements HoodActuatorIO {
  private final Servo actuatorLeft = new Servo(LauncherConstants.ACTUATOR_LEFT_ID);
  private final Servo actuatorRight = new Servo(LauncherConstants.ACTUATOR_RIGHT_ID);

  public HoodActuatorIOHardware() {
    actuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void updateInputs(HoodActuatorIOInputs inputs) {
    inputs.position = (actuatorLeft.getPosition() + actuatorRight.getPosition()) / 2;
  }
}
