package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;

public class HopperCommands {
  /* Set the hopper to the specified mode */
  public static Command setHopperMode(Hopper hopper, HopperConstants.RunMode mode) {
    return Commands.runOnce(() -> hopper.runInMode(mode), hopper);
  }
}