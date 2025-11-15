package frc.robot.commands.pipeline;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedHashSet;
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

  private record Layer(String label, UnaryOperator<DriveInput> operator) {}

  private final Drive drive;

  private final LinkedHashSet<Layer> layers = new LinkedHashSet<>();

  /**
   * Creates a new {@link DriveInputPipeline} with the given base {@link DriveInput}.
   *
   * @param baseInput The base {@link DriveInput} that subsequent layers will modify.
   */
  public DriveInputPipeline(Drive drive) {
    this.drive = drive;
  }

  /**
   * Activates a layer permanently when the returned command starts.
   *
   * <p>When the layer is activated, it is applied to the base input after all previously activated
   * layers.
   *
   * @param operator A function that returns a new {@link DriveInput} with additional behavior.
   * @return A command that activates the layer once when scheduled.
   */
  public Command activatePermanentLayer(String name, UnaryOperator<DriveInput> operator) {
    Layer layer = new Layer(name, operator);
    return Commands.runOnce(() -> activate(layer))
        .andThen(Commands.idle())
        .ignoringDisable(true)
        .withName("Activate Permanent Layer " + name);
  }

  /**
   * Activates a layer for the duration of the returned command. The layer will be deactivated when
   * the command ends.
   *
   * <p>When the layer is activated, it is applied to the base input after all previously activated
   * layers.
   *
   * @param operator A function that returns a new {@link DriveInput} with additional behavior.
   * @return A command that activates the layer while it is running.
   */
  public Command activateLayer(String name, UnaryOperator<DriveInput> operator) {
    Layer layer = new Layer(name, operator);
    return Commands.startEnd(() -> activate(layer), () -> deactivate(layer))
        .withName("Activate Layer " + name);
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

    for (Layer layer : layers) {
      input = layer.operator.apply(input);
    }

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
    return layers.stream().map(Layer::label).toList();
  }

  private void activate(Layer layer) {
    layers.add(layer);
  }

  private void deactivate(Layer layer) {
    layers.remove(layer);
  }
}
