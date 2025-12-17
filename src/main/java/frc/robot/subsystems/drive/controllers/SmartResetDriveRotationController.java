package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.VirtualSubsystem;
import java.util.function.Supplier;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class SmartResetDriveRotationController extends DriveRotationController {

  private Supplier<Rotation2d> defaultGoalSupplier = null;

  // private final Timer resetTimer = new Timer();

  private int cyclesSinceUpdate = 0;

  @SuppressWarnings("unused")
  private final VirtualSubsystem resetter =
      new VirtualSubsystem() {
        @Override
        public void periodic() {
          cyclesSinceUpdate++;
        }
      };

  public SmartResetDriveRotationController(Drive drive, Supplier<Rotation2d> defaultGoalSupplier) {
    super(drive);
    this.defaultGoalSupplier = defaultGoalSupplier;
  }

  @Override
  public void reset() {
    cyclesSinceUpdate = 0;
    super.reset();
    if (defaultGoalSupplier != null) {
      setGoal(defaultGoalSupplier.get());
    }
  }

  @Override
  public double calculate() {
    if (cyclesSinceUpdate > 1) {
      reset();
    }
    cyclesSinceUpdate = 0;
    return super.calculate();
  }
}
