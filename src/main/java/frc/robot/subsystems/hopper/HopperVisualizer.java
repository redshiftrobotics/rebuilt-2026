package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

    private final String name;

    public HopperVisualizer(String name, Color color, double shift) {

        this.name = name;

        mechanism = new LoggedMechanism2d(1, 1);

        feederRoot = mechanism.getRoot("Feeder", .1, .1 + shift);
        feederMech = new LoggedMechanismLigament2d("Feeder", 0, 0, 5, new Color8Bit(color));
        feederRoot.append(feederMech);

        lifterRoot = mechanism.getRoot("Lifter", .9 + shift, .1);
        lifterMech = new LoggedMechanismLigament2d("Lifter", 0, 90, 5, new Color8Bit(color));
        lifterRoot.append(lifterMech);
    }

    public void update(double feedVelocityRadPerSec, double lifterVelocityRadPerSecond) {
        feederMech.setLength(feedVelocityRadPerSec / HopperConstants.FEEDER_MOTOR.freeSpeedRadPerSec * 0.8);
        lifterMech.setLength(lifterVelocityRadPerSecond / HopperConstants.LIFTER_MOTOR.freeSpeedRadPerSec * 0.8);
        Logger.recordOutput(name, mechanism);
    }
}
