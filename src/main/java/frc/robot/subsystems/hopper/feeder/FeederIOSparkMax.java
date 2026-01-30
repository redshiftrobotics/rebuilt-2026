package frc.robot.subsystems.hopper.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.hopper.HopperConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;


/*unfinished, needs...
 stop, updateinputs
*/
public class FeederIOSparkMax implements FeederIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    

    public FeederIOSparkMax() {

        // save the config
        motor = new SparkMax(HopperConstants.FEEDER_CAN_ID, MotorType.kBrushless);
        
        // Get motor resources
        pidController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        //Config object setup 
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(kP, kI, kD);

        //And apply the config
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }    

    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        pidController.setSetpoint( 
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage);
    }

    
}