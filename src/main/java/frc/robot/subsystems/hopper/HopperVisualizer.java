package frc.robot.subsystems.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.common.motorio.MotorIO.MotorIOInputs;
import frc.robot.subsystems.hopper.HopperMotorIO.HopperMotorIOInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HopperVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d feederRoot;
  private final LoggedMechanismRoot2d lifterRoot;
  private final LoggedMechanismLigament2d feederMech;
  private final LoggedMechanismLigament2d lifterMech;

  public HopperVisualizer(Color feederColor, Color lifterColor) {
    mechanism = new LoggedMechanism2d(.256, .256);
    feederRoot = mechanism.getRoot("Feeder", .1, .2);
    feederMech = new LoggedMechanismLigament2d("Feeder", .1, 0, 5, new Color8Bit(feederColor));
    feederRoot.append(feederMech);

    lifterRoot = mechanism.getRoot("Lifter", -.2, .6);
    lifterMech = new LoggedMechanismLigament2d("Lifter", .1, 0, 5, new Color8Bit(lifterColor));
    lifterRoot.append(lifterMech);
  }

  public void think(MotorIOInputs feederInputs, HopperMotorIOInputs lifterInputs) {
    feederMech.setAngle(Units.radiansToDegrees(feederInputs.positionRad));
    lifterMech.setAngle(Units.radiansToDegrees(lifterInputs.positionRad));
    Logger.recordOutput("Hopper/Visualization", mechanism);
  }
}
