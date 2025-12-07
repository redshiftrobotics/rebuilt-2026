package frc.robot.utility.tunable;

import frc.robot.utility.records.FeedforwardConstants;
import frc.robot.utility.records.PIDConstants;
import java.util.function.Consumer;

/**
 * A group of tunable numbers that can be retrieved together.
 *
 * @param <T> The type of the group.
 */
public abstract class TunableNumbers<T> {

  public static class TunablePID extends TunableNumbers<PIDConstants> {
    public TunablePID(String keyPrefix, PIDConstants defaultValues) {
      super(
          new String[] {
            keyPrefix + "/kP", keyPrefix + "/kI", keyPrefix + "/kD",
          },
          new double[] {
            defaultValues.kP(), defaultValues.kI(), defaultValues.kD(),
          });
    }

    @Override
    public PIDConstants get() {
      double[] values = getValues();
      return new PIDConstants(values[0], values[1], values[2]);
    }
  }

  public static class TunableFF extends TunableNumbers<FeedforwardConstants> {
    public TunableFF(String keyPrefix, FeedforwardConstants defaultValues) {
      super(
          new String[] {
            keyPrefix + "/kS", keyPrefix + "/kV", keyPrefix + "/kA",
          },
          new double[] {
            defaultValues.kS(), defaultValues.kV(), defaultValues.kA(),
          });
    }

    @Override
    public FeedforwardConstants get() {
      double[] values = getValues();
      return new FeedforwardConstants(values[0], values[1], values[2]);
    }
  }

  public static class TunableDoubleArray extends TunableNumbers<double[]> {
    public TunableDoubleArray(String[] keys, double[] defaultValues) {
      super(keys, defaultValues);
    }

    @Override
    public double[] get() {
      return getValues();
    }
  }

  private final TunableNumber[] numbers;

  private TunableNumbers(String[] keys, double[] defaultValues) {
    if (keys.length != defaultValues.length) {
      throw new IllegalArgumentException("Keys and default values must have the same length");
    }
    this.numbers = new TunableNumber[keys.length];
    for (int i = 0; i < keys.length; i++) {
      this.numbers[i] = new TunableNumber(keys[i], defaultValues[i]);
    }
  }

  public double[] getValues() {
    double[] values = new double[numbers.length];
    for (int i = 0; i < numbers.length; i++) {
      values[i] = numbers[i].get();
    }
    return values;
  }

  public abstract T get();

  public boolean hasChanged(int id) {
    for (TunableNumber number : numbers) {
      if (number.hasChanged(id)) {
        return true;
      }
    }
    return false;
  }

  public void ifChanged(int id, Runnable onChanged) {
    if (hasChanged(id)) {
      onChanged.run();
    }
  }

  public void ifChanged(int id, Consumer<T> onChanged) {
    if (hasChanged(id)) {
      onChanged.accept(get());
    }
  }
}
