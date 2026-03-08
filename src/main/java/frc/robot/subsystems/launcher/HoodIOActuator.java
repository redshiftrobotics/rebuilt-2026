package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

/** Hardware implementation of the Hood IO. */
public class HoodIOActuator implements HoodIO {

  private final Servo actuatorLeft = new Servo(LauncherConstants.ACTUATOR_LEFT_ID);
  private final Servo actuatorRight = new Servo(LauncherConstants.ACTUATOR_RIGHT_ID);

  public HoodIOActuator() {
    // See Page 9, Sample code
    // https://wcproducts.info/files/frc/manuals/WCP%20Miniature%20Linear%20Servo%20Actuators%20-%20User%20Guide.pdf
    actuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    actuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void setPosition(double position) {
    // TODO, FOR TESTING SETUP ONLY. Do not remove until hood v2 mounted.
    position = MathUtil.clamp(position, 0.1, 0.55);
    actuatorLeft.set(position);
    actuatorRight.set(position);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.isAdjustable = true;

    // Note, this returns the commanded position, not the position that the servo is actually at, as
    // the servo does not report its own position.
    inputs.positionLeft = actuatorLeft.get();
    inputs.positionRight = actuatorRight.get();

    // TODO: Use slew rate limiter to estimate real position
  }
}
