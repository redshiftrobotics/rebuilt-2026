package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.Launcher;

public class LaunchCommands {

  public static Command PrimeToLaunch(Drive drive, Launcher launcher) {
    return Commands.parallel(
        DriveCommands.rotateWithRotationController(drive, () -> launcher.getRobotYaw()),
        launcher.runOnce(
            () -> {
              launcher.start();
            }));
  }
}
