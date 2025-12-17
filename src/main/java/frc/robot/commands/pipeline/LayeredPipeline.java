package frc.robot.commands.pipeline;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * A generic pipeline for modifying data using layers of {@link UnaryOperator}. Each layer can
 * modify the data, and layers can be activated or deactivated.
 *
 * <p>The pipeline starts with base data from a supplier, and each active layer is applied in the
 * order they were activated, with the most recently activated layer being applied last.
 *
 * <p>This allows for multiple modifications to be applied in a modular way without commands wanting
 * exclusive control.
 *
 * @param <T> The type of data being modified by the pipeline.
 */
public class LayeredPipeline<T> {

  private record Layer<T>(String label, UnaryOperator<T> operator) {}

  private final Supplier<T> baseSupplier;
  private final List<Layer<T>> layers = new ArrayList<>();

  /**
   * Creates a new pipeline with a supplier for the base data.
   *
   * @param baseSupplier Supplier that provides fresh base data each time the pipeline is evaluated.
   */
  public LayeredPipeline(Supplier<T> baseSupplier) {
    this.baseSupplier = baseSupplier;
  }

  /**
   * Activates a layer for the duration of the returned command. The layer will be deactivated when
   * the command ends.
   *
   * @param name A label for the layer, used for debugging and display purposes.
   * @param operator A function that modifies the data.
   * @return A command that activates the layer while it is running.
   */
  public Command runLayer(String name, UnaryOperator<T> operator) {
    Layer<T> layer = new Layer<>(name, operator);
    return Commands.startEnd(() -> activate(layer), () -> deactivate(layer))
        .withName("Activate Layer " + name);
  }

  /**
   * Gets the current output of the pipeline by applying all active layers to fresh base data.
   *
   * @return The modified data after all layers have been applied.
   */
  public T get() {
    T result = baseSupplier.get();
    for (Layer<T> layer : layers) {
      result = layer.operator.apply(result);
    }
    return result;
  }

  /**
   * Gets all active layers' labels, with the most recently activated (and last applied) layer at
   * the end of the list.
   *
   * @return A list of labels of all active layers.
   */
  public List<String> getActiveLayers() {
    return layers.stream().map(Layer::label).toList();
  }

  private void activate(Layer<T> layer) {
    layers.add(layer);
  }

  private void deactivate(Layer<T> layer) {
    layers.remove(layer);
  }
}
