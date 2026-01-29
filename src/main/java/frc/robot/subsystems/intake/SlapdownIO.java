package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
    @AutoLog
    public class SlapdownIOInputsAutoLogged {
    public double PositionRad = 0.0;
    public double VelocityRadPerSec = 0.0;

    public double[] AppliedVolts = new double[] {};
    public double[] SupplyCurrentAmps = new double[] {};
  }

    public default void updateInputs(SlapdownIOInputsAutoLogged inputs){

    }
}
