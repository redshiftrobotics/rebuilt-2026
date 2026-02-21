package frc.robot.subsystems.common.velocityMotor;

public record MotorConstants(
    int deviceId, double gearRatio, boolean inverted, boolean brakeMode, double stallCurrent) {}
