package frc.robot.utility.records;

import com.ctre.phoenix6.configs.Slot0Configs;

public record PIDConfig(double kP, double kI, double kD) {
  public com.pathplanner.lib.config.PIDConstants toPathPlannerPIDConstants() {
    return new com.pathplanner.lib.config.PIDConstants(kP, kI, kD);
  }

  public PIDConfig(Slot0Configs slot0Configs) {
    this(slot0Configs.kP, slot0Configs.kI, slot0Configs.kD);
  }
}
