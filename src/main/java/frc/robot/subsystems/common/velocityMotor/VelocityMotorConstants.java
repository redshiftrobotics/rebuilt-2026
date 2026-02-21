package frc.robot.subsystems.common.velocityMotor;

public record VelocityMotorConstants(
    int deviceId, double gearRatio, boolean inverted, boolean brakeMode, double stallCurrent) {}
