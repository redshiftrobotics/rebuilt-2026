package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;

/** Hardware implementation of the TemplateIO. */
public class HoodIOActuator implements HoodIO {

  private final Servo actuatorLeft = new Servo(LauncherConstants.ACTUATOR_LEFT_ID);
  private final Servo actuatorRight = new Servo(LauncherConstants.ACTUATOR_RIGHT_ID);

  public HoodIOActuator() {
    actuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    actuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  public void setPosition(double position) {
    actuatorLeft.set(position);
    actuatorRight.set(position);
  }

  private double getPosition() {
    return (actuatorLeft.getPosition() + actuatorRight.getPosition()) / 2;
  }

  @Override
  public void setAngle(Rotation2d angle) {
    Distance r = LauncherConstants.HOOD_RADIUS;
    // Convert launch angle to hood angle
    angle = angle.plus(Rotation2d.kCCW_90deg);
    Translation2d hoodPosition =
        new Translation2d(r.times(angle.getCos()), r.times(angle.getSin()));

    double lengthMeters = hoodPosition.getDistance(LauncherConstants.ACTUATOR_LOCATION);

    double position =
        (lengthMeters - LauncherConstants.ACTUATOR_LENGTH_MIN.in(Meters))
            / (LauncherConstants.ACTUATOR_EXTENSION.in(Meters));
    setPosition(position);
  }

  public Rotation2d getAngle() {
    double actuatorLength =
        getPosition() * LauncherConstants.ACTUATOR_EXTENSION.in(Meters)
            + LauncherConstants.ACTUATOR_LENGTH_MIN.in(Meters);
    double radius = LauncherConstants.HOOD_RADIUS.in(Meters);
    double actuatorDistance = LauncherConstants.ACTUATOR_LOCATION.getNorm();

    // https://www.desmos.com/calculator/pkbsecs465
    // https://www.youtube.com/watch?v=Qji5x8gBVX4
    Rotation2d relativeAngle =
        new Rotation2d(
            2 * Math.PI
                - Math.acos(
                    (radius * radius
                            + actuatorDistance * actuatorDistance
                            - (actuatorLength * actuatorDistance))
                        / (2 * radius * actuatorDistance)));
    // Convert from relative angle to absolute angle, then to launch angle
    return relativeAngle
        .plus(LauncherConstants.ACTUATOR_LOCATION.getAngle())
        .minus(Rotation2d.kCCW_90deg);
  }

  @Override
  public HoodType hoodType() {
    return HoodType.FIXED;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angleRadians = getAngle().getRadians();
  }
}
