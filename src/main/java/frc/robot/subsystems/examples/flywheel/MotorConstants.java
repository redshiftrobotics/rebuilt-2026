package frc.robot.subsystems.examples.flywheel;

public record MotorConstants(
        int deviceId, double gearRatio, boolean inverted, boolean brakeMode, double stallCurrent) {}
