package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeConstants.IntakeWheelConstants;
import frc.robot.utility.SparkUtil;

/** SparkMAX implementation of IntakeWheelIO. */
public class IntakeWheelIOSparkMax implements IntakeWheelIO {
    private final SparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final Debouncer connectionDebouncer = new Debouncer(0.5);

    public IntakeWheelIOSparkMax() {
        this.motor = new SparkMax(IntakeWheelConstants.CAN_ID, MotorType.kBrushless);

        relativeEncoder = motor.getEncoder();

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(IntakeWheelConstants.BRAKE_MODE ? IdleMode.kBrake : IdleMode.kCoast)
                .inverted(IntakeWheelConstants.INVERTED)
                .smartCurrentLimit(40)
                .voltageCompensation(12);

        config.encoder
                .positionConversionFactor(IntakeWheelConstants.GEAR_RATIO)
                .velocityConversionFactor(IntakeWheelConstants.GEAR_RATIO);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeWheelIOInputsAutoLogged inputs) {
        SparkUtil.clearError();

        SparkUtil.ifOk(
                motor, relativeEncoder::getPosition, value -> inputs.positionRad = Units.rotationsToRadians(value));
        SparkUtil.ifOk(
                motor,
                relativeEncoder::getVelocity,
                value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
        SparkUtil.ifOk(
                motor, () -> motor.getAppliedOutput() * motor.getBusVoltage(), values -> inputs.appliedVolts = values);
        SparkUtil.ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);

        inputs.motorConnected = connectionDebouncer.calculate(!SparkUtil.hasError());
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
