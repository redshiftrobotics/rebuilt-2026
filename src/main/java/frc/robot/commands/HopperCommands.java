package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;

public class HopperCommands {
  /* Set the hopper to the specified mode */
  public static Command setHopperMode(Hopper hopper, HopperConstants.HopperRunMode mode) {
    return Commands.runOnce(() -> hopper.setMode(mode), hopper);
  }

  /* Runs a hopper test routine */
  public static Command hopperTestRoutine(Hopper hopper) {
    return setHopperMode(hopper, HopperConstants.HopperRunMode.IDLE)
        .andThen(Commands.waitSeconds(5))
        .andThen(setHopperMode(hopper, HopperConstants.HopperRunMode.STOPPED))
        .andThen(Commands.waitSeconds(2))
        .andThen(setHopperMode(hopper, HopperConstants.HopperRunMode.FIRING))
        .andThen(Commands.waitSeconds(5))
        .andThen(setHopperMode(hopper, HopperConstants.HopperRunMode.STOPPED))
        .andThen(Commands.waitSeconds(2))
        .andThen(setHopperMode(hopper, HopperConstants.HopperRunMode.REVERSE))
        .andThen(Commands.waitSeconds(5))
        .andThen(setHopperMode(hopper, HopperConstants.HopperRunMode.STOPPED));
  }
}
