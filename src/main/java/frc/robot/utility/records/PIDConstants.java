package frc.robot.utility.records;

public record PIDConstants(double kP, double kI, double kD) {
  public com.pathplanner.lib.config.PIDConstants toPathPlannerPIDConstants() {
    return new com.pathplanner.lib.config.PIDConstants(kP, kI, kD);
  }
}
