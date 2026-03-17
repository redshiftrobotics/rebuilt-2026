package frc.robot.debugRobots;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ServoTestRobot extends TimedRobot {

  private final Servo servo = new Servo(0);

  double setpoint = 0;

  public ServoTestRobot() {
    servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    SmartDashboard.putData(
        "Servo State",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Setpoint", servo::get, servo::set);
            builder.addDoubleProperty("Position", servo::getPosition, servo::setPosition);
          }
        });
  }

  @Override
  public void robotPeriodic() {}
}
