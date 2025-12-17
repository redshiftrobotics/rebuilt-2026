package frc.robot.commands.pipeline;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.DriveRotationController;
import frc.robot.subsystems.drive.controllers.SmartResetDriveRotationController;
import java.util.StringJoiner;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * A pipeline for modifying {@link DriveInput} using layers.
 *
 * <p>This wraps a {@link LayeredPipeline} with drive-specific functionality.
 */
public class DriveInputPipeline {

  private final LayeredPipeline<DriveInput> pipeline;
  private final DriveRotationController headingController;

  public DriveInputPipeline(Drive drive) {
    this(drive, () -> new DriveInput(drive));
  }

  public DriveInputPipeline(Drive drive, Supplier<DriveInput> baseSupplier) {
    this.headingController =
        new SmartResetDriveRotationController(drive, () -> drive.getRobotPose().getRotation());
    this.pipeline = new LayeredPipeline<>(baseSupplier);
  }

  /**
   * Activates a layer for the duration of the returned command.
   *
   * @param name A label for the layer.
   * @param operator A function that modifies the drive input.
   * @return A command that activates the layer while it is running.
   */
  public Command runLayer(String name, UnaryOperator<DriveInput> operator) {
    return pipeline.runLayer(name, operator);
  }

  /**
   * Gets the current {@link ChassisSpeeds} output of the pipeline.
   *
   * @return The current {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return pipeline.get().getChassisSpeeds(headingController);
  }

  /**
   * Gets all active layers' labels.
   *
   * @return A list of labels of all active layers.
   */
  public String getLayerInfo() {
    StringJoiner joiner = new StringJoiner(" > ");
    joiner.add("Drive");
    pipeline.getActiveLayers().forEach(joiner::add);
    return joiner.toString();
  }
}
