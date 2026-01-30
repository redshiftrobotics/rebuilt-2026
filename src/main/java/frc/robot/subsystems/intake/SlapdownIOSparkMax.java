package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SlapdownIOSparkMax implements SlapdownIO {

    private final SparkMax motor;
    private final SparkClosedLoopController motorPID;

    public SlapdownIOSparkMax(SparkMax motor){
     this.motor = motor;   
     motorPID = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(SlapdownIOInputsAutoLogged inputs) {
        
    }

    @Override
    public void setMotorMode() {
        
    }

    @Override
    public void setPID(double kp, double ki,double kd) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(kp, ki, kd);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }
}
