package frc.robot.subsystems.outtake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.utility.tunable.TunableNumbers.TunableFF;
import frc.robot.utility.tunable.TunableNumbers.TunablePID;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {

  public final TunablePID pidConfig = new TunablePID(getName() + "/PID", OuttakeConstants.PID);
  public final TunableFF ffConfig = new TunableFF(getName() + "/FF", OuttakeConstants.FF);

  private final FlywheelIO left;
  private final FlywheelIO middle;
  private final FlywheelIO right;
  private final List<FlywheelIO> motors;

  private final FlywheelIOInputsAutoLogged leftInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged middleInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged rightInputs = new FlywheelIOInputsAutoLogged();

  private final Alert leftMotorDisconnectedAlert =
      new Alert("Left flywheel motor disconnected, ", AlertType.kError);
  private final Alert middleMotorDisconnectedAlert =
      new Alert("Middle flywheel motor disconnected", AlertType.kError);
  private final Alert rightMotorDisconnectedAlert =
      new Alert("Right flywheel motor disconnected", AlertType.kError);
  private final Alert leftMotorConfigFailAlert =
      new Alert("Left flywheel motor config fail, ", AlertType.kError);
  private final Alert middleMotorConfigFailAlert =
      new Alert("Middle flywheel motor config fail", AlertType.kError);
  private final Alert rightMotorConfigFailAlert =
      new Alert("Right flywheel motor config fail", AlertType.kError);

  private boolean running = false;
  private double desiredRadPerSec = 9;

  public Outtake(FlywheelIO left, FlywheelIO middle, FlywheelIO right) {

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
    ffConfig.ifChanged(hashCode(), this::updateFF);

    leftMotorDisconnectedAlert.set(!leftInputs.motorConnected);
    middleMotorDisconnectedAlert.set(!middleInputs.motorConnected);
    rightMotorDisconnectedAlert.set(!rightInputs.motorConnected);
    leftMotorConfigFailAlert.set(leftInputs.pushedConfigFault);
    middleMotorConfigFailAlert.set(middleInputs.pushedConfigFault);
    rightMotorConfigFailAlert.set(rightInputs.pushedConfigFault);
  }

  public Command runFlywheelsCommand() {
    return startEnd(this::runFlywheels, this::stopFlywheels);
  }

  public void stopFlywheels() {
    running = false;
    motors.forEach(FlywheelIO::stop);
  }

  public void runFlywheels() {
    running = true;
    motors.forEach(m -> m.setVelocity(desiredRadPerSec));
    // motors.forEach(m -> m.setDutyCycle(desiredRadPerSec / 100.0));
  }

  public double getRunningDesiredRadPerSec() {
    return desiredRadPerSec;
  }

  public void setRunningDesiredRadPerSec(double desiredRadPerSec) {
    this.desiredRadPerSec = desiredRadPerSec;
    if (running) {
      runFlywheels();
    }
  }

  private void updatePID() {
    motors.forEach(m -> m.setPID(pidConfig.get()));
  }

  private void updateFF() {
    motors.forEach(m -> m.setFF(ffConfig.get()));
  }

  public static Outtake create(RobotType robotType) {
    switch (robotType) {
      case REBUILT_2026:
        return new Outtake(
            new FlywheelIOTalonFX(OuttakeConstants.LEFT_CONSTANTS),
            new FlywheelIOTalonFX(OuttakeConstants.MIDDLE_CONSTANTS),
            // new FlywheelIOTalonFX(OuttakeConstants.RIGHT_CONSTANTS));
            new FlywheelIO() {});
      case SIM_BOT:
        return new Outtake(
            new FlywheelIOSim(OuttakeConstants.LEFT_CONSTANTS),
            new FlywheelIOSim(OuttakeConstants.MIDDLE_CONSTANTS),
            new FlywheelIOSim(OuttakeConstants.RIGHT_CONSTANTS));
      default:
        return new Outtake(new FlywheelIO() {}, new FlywheelIO() {}, new FlywheelIO() {});
    }
  }
}
