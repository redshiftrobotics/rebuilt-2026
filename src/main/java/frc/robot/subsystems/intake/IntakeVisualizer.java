package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.IntakeConstants.SlapdownConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeVisualizer {

  private final LoggedMechanism2d visualizerMechanism = new LoggedMechanism2d(3, 1.5);
  private final LoggedMechanismRoot2d visualizerRoot =
      visualizerMechanism.getRoot("wheel", 1.75, .25);
  private final LoggedMechanismLigament2d mechanism;
  private final LoggedMechanismLigament2d wheelArm1;
  private final LoggedMechanismLigament2d wheelArm2;

  private final String name;

  public IntakeVisualizer(String name, Color color) {

    this.name = name;

    mechanism =
        visualizerRoot.append(
            new LoggedMechanismLigament2d(
                "slapdownArm",
                0.5,
                SlapdownConstants.UP_SETPOINT.getDegrees(),
                10,
                new Color8Bit(color)));

    wheelArm1 =
        mechanism.append(
            new LoggedMechanismLigament2d("wheelArm1", 0.09, 90, 9, new Color8Bit(color)));
    wheelArm2 =
        mechanism.append(
            new LoggedMechanismLigament2d("wheelArm2", 0.09, 180, 9, new Color8Bit(color)));
  }

  public void update(double slapdownPositionRad, double wheelPositionRad) {
    mechanism.setAngle(Units.radiansToDegrees(slapdownPositionRad));
    wheelArm1.setAngle(Units.radiansToDegrees(wheelPositionRad));
    wheelArm2.setAngle(Units.radiansToDegrees(wheelPositionRad + Math.PI));

    Logger.recordOutput(name, visualizerMechanism);
  }
}
