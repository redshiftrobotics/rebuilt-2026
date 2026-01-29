package frc.robot.subsystems.intake;

import java.time.Period;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    private final IntakeWheelIO wheelIO;
    private final SlapdownIO SlapdownIO;

    public Intake(IntakeWheelIO wheelIO, SlapdownIO slapdownIO){
        this.wheelIO = wheelIO;
        this.SlapdownIO = slapdownIO;
    }

    @Override
    public void periodic() {
        
    }   
}
