package frc.robot.utility.records;

import com.ctre.phoenix6.configs.Slot0Configs;

public record FeedForwardConfigRecord(double kS, double kV, double kA) {
  public FeedForwardConfigRecord(Slot0Configs slot0Configs) {
    this(slot0Configs.kS, slot0Configs.kV, slot0Configs.kA);
  }
}
