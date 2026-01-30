package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;

public class HopperCommands {
    private static HopperConstants.RunMode currentMode;

    /* Set the hopper to the specified mode */
    public static Command setHopperMode(Hopper hopper, HopperConstants.RunMode mode) {
        currentMode = mode;
        return Commands.runOnce(() -> {
            hopper.runBubblerAtVelocity(mode.bubblerVelocityRadPerSec);
            hopper.runFeederAtVelocity(mode.feederVelocityRadPerSec);
        }, hopper);
    }

    /* Set the hopper to the specified mode for a certain time, then return to the old mode */
    public static Command setHopperModeTimed(Hopper hopper, HopperConstants.RunMode mode, double timeSeconds) {
        HopperConstants.RunMode oldMode = currentMode;
        return setHopperMode(hopper, mode).andThen(Commands.waitSeconds(timeSeconds)).andThen(setHopperMode(hopper, oldMode));
    }
}
