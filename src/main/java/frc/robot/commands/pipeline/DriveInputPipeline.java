package frc.robot.commands.pipeline;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedList;
import java.util.List;
import java.util.function.UnaryOperator;

/**
 * A pipeline for modifying {@link DriveInput} using layers of {@link UnaryOperator}. Each layer can
 * modify the input, and layers can be activated or deactivated.
 *
 * <p>The pipeline starts with a base {@link DriveInput}, and each active layer is applied in the
 * order they were activated, with the most recently activated layer being applied last.
 *
 * <p>This allows for multiple modifications to be applied to the drive input in a modular way, such
 * as adding a slow mode layer, combined with an aiming layer, without running into the issue of
 * commands wanting exclusive control over the drive input.
 */
public class DriveInputPipeline {

  private final Drive drive;

  private final List<UnaryOperator<DriveInput>> layers = new LinkedList<>();

  private DriveInput lastInput = null;

  /**
   * Creates a new {@link DriveInputPipeline} with the given base {@link DriveInput}.
   *
   * @param baseInput The base {@link DriveInput} that subsequent layers will modify.
   */
  public DriveInputPipeline(Drive drive) {
    this.drive = drive;
  }

  /**
   * Activates a layer for the duration of the returned command. The layer will be deactivated when
   * the command ends.
   *
   * <p>When the layer is activated, it is applied to the base input after all previously activated
   * layers.
   *
   * @param layer A function that returns a new {@link DriveInput} with additional behavior.
   * @return A command that activates the layer while it is running.
   */
  public Command activateLayer(UnaryOperator<DriveInput> layer) {
    return Commands.startEnd(() -> activate(layer), () -> deactivate(layer))
        .ignoringDisable(true)
        .withName("Activate Layer " + layer.hashCode());
  }

  /** Clears all active modifying layers. */
  public void clearLayers() {
    layers.clear();
  }

  /**
   * Gets the current {@link ChassisSpeeds} output of the pipeline.
   *
   * <p>The speed comes from the base input modified by all active layers.
   *
   * @return The current {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getChassisSpeeds() {
    DriveInput input = new DriveInput(drive);

    for (UnaryOperator<DriveInput> layer : layers) {
      input = layer.apply(input);
    }

    this.lastInput = input;

    return input.getChassisSpeeds();
  }

  /**
   * Gets all active layers' labels, with the most recently activated (and last applied) layer at
   * the end of the list.
   *
   * <p>This can be used for debugging or displaying the current state of the pipeline.
   *
   * @return A list of labels of all active layers.
   */
  public List<String> getActiveLayers() {
    if (lastInput == null) {
      return List.of();
    }
    return lastInput.getLabels();
  }

  private void activate(UnaryOperator<DriveInput> layer) {
    layers.add(layer);
  }

  private void deactivate(UnaryOperator<DriveInput> layer) {
    layers.remove(layer);
  }
}
