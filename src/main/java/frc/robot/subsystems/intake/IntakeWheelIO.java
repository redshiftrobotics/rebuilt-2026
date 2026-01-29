package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelIO {
  @AutoLog
  public class IntakeIOInputsAutoLogged {
    public double leftPositionRad = 0.0;
    public double rightPositionRad = 0.0;
    public double leftVelocity = 0.0;
    public double rightVelocity = 0.0;

    public double[] leftAppliedVolts = new double[] {};
    public double[] rightAppliedVolts = new double[] {};
    public double[] leftSupplyCurrentAmps = new double[] {};
    public double[] rightSupplyCurrentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  public default void setVelocity(double velocity) {}

  public default void stop() {}
}