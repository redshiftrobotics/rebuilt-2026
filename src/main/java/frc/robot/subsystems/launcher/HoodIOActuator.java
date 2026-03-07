package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.Servo;

/** Hardware implementation of the Hood IO. */
public class HoodIOActuator implements HoodIO {

  private final Servo actuatorLeft = new Servo(LauncherConstants.ACTUATOR_LEFT_ID);
  private final Servo actuatorRight = new Servo(LauncherConstants.ACTUATOR_RIGHT_ID);

  public HoodIOActuator() {
    actuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    actuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void setPosition(double position) {
    actuatorLeft.set(position);
    actuatorRight.set(position);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.isAdjustable = true;
    inputs.positionLeft = actuatorLeft.get();
    inputs.positionRight = actuatorRight.get();
  }
}
