package frc.robot.subsystems.led;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to manage multiple LED strips with Blinken LED Drivers.
 *
 * <p>LEDs will retain their last set pattern until a new pattern is set, unless a default pattern
 * is given though a default command.
 */
public class LEDSubsystem extends SubsystemBase {

    private LEDStripIO[] strips;
    private LEDStripIOInputsAutoLogged[] inputs;

    private final Debouncer setupDebouncer = new Debouncer(0.8);

    public static LEDSubsystem create(RobotType robotType) {
        switch (robotType) {
            case REBUILT_2026:
                return new LEDSubsystem(
                        new LEDStripIOBlinkin(LEDConstants.LED_STRIP_BACK, BlinkinLEDPattern.OFF),
                        new LEDStripIOBlinkin(LEDConstants.LED_STRIP_1X1, BlinkinLEDPattern.OFF),
                        new LEDStripIOBlinkin(LEDConstants.LED_STRIP_FUTURE, BlinkinLEDPattern.OFF));

            case SIM_BOT:
                return new LEDSubsystem(new LEDStripIOSim(BlinkinLEDPattern.OFF));

            default:
                return new LEDSubsystem();
        }
    }

    private LEDSubsystem(LEDStripIO... strips) {
        this.strips = strips;

        inputs = Arrays.stream(strips)
                .map(s -> new LEDStripIOInputsAutoLogged())
                .toArray(LEDStripIOInputsAutoLogged[]::new);
    }

    @Override
    public void periodic() {
        boolean runSetup = !setupDebouncer.calculate(DriverStation.isEnabled()) && DriverStation.isEnabled();

        for (int i = 0; i < strips.length; i++) {
            strips[i].runSetup(runSetup);
            strips[i].updateInputs(inputs[i]);
            Logger.processInputs("LED/strip" + i, inputs[i]);
        }
    }

    public Command runColor(BlinkinLEDPattern color) {
        return run(() -> set(color)).withName("LED " + color);
    }

    public Command runColor(Supplier<BlinkinLEDPattern> color) {
        return run(() -> set(color.get())).withName("LED Supplied Color");
    }

    public Command runColor(
            BlinkinLEDPattern colorIfBlue, BlinkinLEDPattern colorIfRed, BlinkinLEDPattern colorIfUnknown) {
        return runColor(() -> DriverStation.getAlliance()
                .map(a -> a == Alliance.Blue ? colorIfBlue : colorIfRed)
                .orElse(colorIfUnknown));
    }

    public Command runNoColor() {
        return runColor(BlinkinLEDPattern.OFF);
    }

    public void set(BlinkinLEDPattern pattern) {
        for (int i = 0; i < strips.length; i++) {
            set(i, pattern);
        }
        Logger.recordOutput("LED/setPattern", pattern.toString());
    }

    public void set(int stripIndex, BlinkinLEDPattern pattern) {
        strips[stripIndex].setPattern(pattern);
    }
}
