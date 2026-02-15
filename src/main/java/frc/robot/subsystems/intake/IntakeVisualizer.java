package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utility.VirtualSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeVisualizer extends VirtualSubsystem {

  private final LoggedMechanism2d visualizerMechanism = new LoggedMechanism2d(3, 1.5);
  private final LoggedMechanismRoot2d visualizerRoot =
      visualizerMechanism.getRoot("wheel", 1.75, .25);
  private final LoggedMechanismLigament2d visualizerSlapdownArm =
      visualizerRoot.append(
          new LoggedMechanismLigament2d(
              "slapdownArm",
              0.5,
              IntakeConstants.SLAPDOWN_UP_SETPOINT.getDegrees(),
              10,
              new Color8Bit(Color.kOrange)));
  private final LoggedMechanismLigament2d wheelArm1 =
      visualizerSlapdownArm.append(
          new LoggedMechanismLigament2d("wheelArm1", 0.09, 90, 10, new Color8Bit(Color.kRed)));
  private final LoggedMechanismLigament2d wheelArm2 =
      visualizerSlapdownArm.append(
          new LoggedMechanismLigament2d("wheelArm2", 0.09, 180, 10, new Color8Bit(Color.kRed)));

  private final String name;
  private DoubleSupplier slapdownPositionRadSupplier;
  private DoubleSupplier wheelPositionRadSupplier;

  public IntakeVisualizer(
      String name, DoubleSupplier slapdownPositionRadSupplier, DoubleSupplier wheelPositionRad) {
    this.name = name;
    this.slapdownPositionRadSupplier = slapdownPositionRadSupplier;
    this.wheelPositionRadSupplier = wheelPositionRad;
  }

  @Override
  public void periodic() {
    visualizerSlapdownArm.setAngle(
        Units.radiansToDegrees(slapdownPositionRadSupplier.getAsDouble()));
    wheelArm1.setAngle(Units.radiansToDegrees(wheelPositionRadSupplier.getAsDouble()));
    wheelArm2.setAngle(Units.radiansToDegrees(wheelPositionRadSupplier.getAsDouble() + Math.PI));

    Logger.recordOutput(name + "/Visualization", visualizerMechanism);
  }
}
