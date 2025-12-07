package frc.robot.utility.records;

import com.ctre.phoenix6.configs.Slot0Configs;

public record FeedforwardConstants(double kS, double kV, double kA) {
  public FeedforwardConstants(Slot0Configs slot0Configs) {
    this(slot0Configs.kS, slot0Configs.kV, slot0Configs.kA);
  }
}
