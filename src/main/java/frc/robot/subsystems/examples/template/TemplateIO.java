package frc.robot.subsystems.examples.template;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface TemplateIO {
    @AutoLog
    public static class TemplateIOInputs {
        public double velocityRadPerSec = 0.0;
        // motor connected?
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(TemplateIOInputs inputs) {}

    /** Run open loop at the specified percentage power. */
    public default void setSpeed(double speed) {}
}
