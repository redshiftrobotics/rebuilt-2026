package frc.robot.utility.tunable;

import frc.robot.utility.records.FeedforwardConstants;
import frc.robot.utility.records.PIDConstants;
import frc.robot.utility.tunable.TunableNumbers.TunableFF;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;

public class TunableNumberGroup {
  private final String key;

  /**
   * Create a new LoggedTunableNumberGroup
   *
   * @param key Key on dashboard
   */
  public TunableNumberGroup(String key) {
    this.key = key;
  }

  // --- Create Number ---

  /**
   * Add a number to the group
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber number(String dashboardKey, double defaultValue) {
    return new TunableNumber(key + "/" + dashboardKey, defaultValue);
  }

  public TunablePID pid(String dashboardKey, PIDConstants defaultValues) {
    return new TunableNumbers.TunablePID(key + "/" + dashboardKey, defaultValues);
  }

  public TunableFF ff(String dashboardKey, FeedforwardConstants defaultValues) {
    return new TunableNumbers.TunableFF(key + "/" + dashboardKey, defaultValues);
  }

  // --- Create Subgroup ---

  /**
   * Add a subgroup to the group
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumberGroup subgroup(String dashboardKey) {
    return new TunableNumberGroup(key + "/" + dashboardKey);
  }
}
