package frc.robot.subsystems.launcher;

/** Simulation implementation of the Hood IO. */
public class HoodIOSim implements HoodIO {

    private final boolean isAdjustable;
    private double position = 0.0;

    public HoodIOSim(boolean isAdjustable) {
        this.isAdjustable = isAdjustable;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void setPosition(double position) {
        if (isAdjustable) {
            this.position = position;
        }
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.isAdjustable = isAdjustable;
        inputs.positionLeft = position;
        inputs.positionRight = position;
    }
}
