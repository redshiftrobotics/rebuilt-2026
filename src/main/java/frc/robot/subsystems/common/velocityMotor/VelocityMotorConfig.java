package frc.robot.subsystems.common.velocityMotor;

public record VelocityMotorConfig(
    int deviceId, double gearRatio, boolean inverted, boolean brakeMode, double stallCurrent) {}
