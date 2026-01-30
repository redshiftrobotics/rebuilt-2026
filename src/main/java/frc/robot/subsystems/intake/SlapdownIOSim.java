package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SlapdownIOSim implements SlapdownIO{

    private final DCMotorSim motor;

    public SlapdownIOSim(DCMotorSim motor) {
        this.motor = motor;
    }

    @Override
    public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
        inputs.PositionRad = motor.getAngularPositionRad();
        inputs.VelocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.AppliedVolts = new double[] {0.0};
        inputs.SupplyCurrentAmps = new double[] {motor.getCurrentDrawAmps()};
    }

    @Override
    public void setMotorMode() {
        
    }

    @Override
    public void setPID(double kp, double ki,double kd) {
        
    }

    @Override
    public void setSpeed(double speed) {
        motor.setInput(speed);
    }

    @Override
    public void stopMotor(){
        motor.setAngularVelocity(0);
    }
}
