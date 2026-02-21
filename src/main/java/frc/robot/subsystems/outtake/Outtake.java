package frc.robot.subsystems.outtake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.common.velocityMotor.MotorIO;
import frc.robot.subsystems.common.velocityMotor.MotorIOInputsAutoLogged;
import frc.robot.subsystems.common.velocityMotor.MotorIOSim;
import frc.robot.subsystems.common.velocityMotor.MotorIOSparkMax;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {

  public final TunablePID pidConfig = new TunablePID(getName() + "/PID", OuttakeConstants.PID);

  private final MotorIO left;
  private final MotorIO middle;
  private final MotorIO right;

  private final MotorIOInputsAutoLogged leftInputs = new MotorIOInputsAutoLogged();
  private final MotorIOInputsAutoLogged middleInputs = new MotorIOInputsAutoLogged();
  private final MotorIOInputsAutoLogged rightInputs = new MotorIOInputsAutoLogged();

  private final Alert leftMotorDisconnectedAlert =
      new Alert("Left motor disconnected, ", AlertType.kError);
  private final Alert middleMotorDisconnectedAlert =
      new Alert("Middle motor disconnected", AlertType.kError);
  private final Alert rightMotorDisconnectedAlert =
      new Alert("Right motor disconnected", AlertType.kError);
  private final List<MotorIO> motors;

  private boolean running = false;
  private double desiredRadPerSec = 15;

  public Outtake(MotorIO left, MotorIO middle, MotorIO right) {

    this.left = left;
    this.middle = middle;
    this.right = right;

    motors = List.of(left, middle, right);
    updatePID();

    SmartDashboard.putData(
        "Outtake State",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("Run", () -> running, null);
            builder.addDoubleProperty(
                "Desired Velocity (rad per sec)",
                () -> desiredRadPerSec,
                v -> setRunningDesiredRadPerSec(v));
            builder.addDoubleProperty(
                "Left Velocity (rad per sec)", () -> leftInputs.velocityRadPerSec, null);
            builder.addDoubleProperty(
                "Middle Velocity (rad per sec)", () -> middleInputs.velocityRadPerSec, null);
            builder.addDoubleProperty(
                "Right Velocity (rad per sec)", () -> rightInputs.velocityRadPerSec, null);
            builder.addDoubleProperty("Left Dutycycle", () -> leftInputs.appliedDutycycle, null);
            builder.addDoubleProperty(
                "Middle Dutycycle", () -> middleInputs.appliedDutycycle, null);
            builder.addDoubleProperty("Right Dutycycle", () -> rightInputs.appliedDutycycle, null);
          }
        });
  }

  @Override
  public void periodic() {
    left.updateInputs(leftInputs);
    middle.updateInputs(middleInputs);
    right.updateInputs(rightInputs);

    Logger.processInputs(getName() + "/Left", leftInputs);
    Logger.processInputs(getName() + "/Middle", middleInputs);
    Logger.processInputs(getName() + "/Right", rightInputs);

    pidConfig.ifChanged(hashCode(), this::updatePID);

    leftMotorDisconnectedAlert.set(!leftInputs.motorConnected);
    middleMotorDisconnectedAlert.set(!middleInputs.motorConnected);
    rightMotorDisconnectedAlert.set(!rightInputs.motorConnected);
  }

  public Command runFlywheelsCommand() {
    return startEnd(this::runFlywheels, this::stopFlywheels);
  }

  public void stopFlywheels() {
    running = false;
    motors.forEach(MotorIO::stop);
  }

  public void runFlywheels() {
    running = true;
    motors.forEach(m -> m.setVelocity(desiredRadPerSec));
  }

  public double getRunningDesiredRadPerSec() {
    return desiredRadPerSec;
  }

  public void setRunningDesiredRadPerSec(double desiredRadPerSec) {
    this.desiredRadPerSec = desiredRadPerSec;
    if (running) {
      motors.forEach(m -> m.setVelocity(desiredRadPerSec));
    }
  }

  private void updatePID() {
    motors.forEach(m -> m.setPID(pidConfig.get()));
  }

  public static Outtake create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Outtake(
            new MotorIOSparkMax(OuttakeConstants.LEFT_CONSTANTS),
            new MotorIOSparkMax(OuttakeConstants.MIDDLE_CONSTANTS),
            new MotorIOSparkMax(OuttakeConstants.RIGHT_CONSTANTS));
      case SIM_BOT:
        return new Outtake(
            new MotorIOSim(OuttakeConstants.MOTOR, OuttakeConstants.MIDDLE_CONSTANTS, 0.1),
            new MotorIOSim(OuttakeConstants.MOTOR, OuttakeConstants.MIDDLE_CONSTANTS, 0.1),
            new MotorIOSim(OuttakeConstants.MOTOR, OuttakeConstants.MIDDLE_CONSTANTS, 0.1));
      default:
        return new Outtake(new MotorIO() {}, new MotorIO() {}, new MotorIO() {});
    }
  }
}
