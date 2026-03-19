package frc.robot.debugRobots;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.ModuleConstants;
import frc.robot.subsystems.drive.ModuleConstants.ModuleConfig;
import java.util.ArrayList;
import java.util.List;

public class BrakeModeTest extends TimedRobot {

  private final List<TalonFX> motors = new ArrayList<>();
  private final List<TalonFXConfiguration> motorConfigs = new ArrayList<>();

  private boolean brakeModeStartsEnabled = true;

  private boolean brakeModeEnabled = false;

  public BrakeModeTest() {
    var configs =
        List.of(
            ModuleConstants.FRONT_LEFT_MODULE_CONFIG,
            ModuleConstants.FRONT_RIGHT_MODULE_CONFIG,
            ModuleConstants.BACK_LEFT_MODULE_CONFIG,
            ModuleConstants.BACK_RIGHT_MODULE_CONFIG);

    for (ModuleConfig config : configs) {
      motors.add(new TalonFX(config.driveID()));
      motorConfigs.add(new TalonFXConfiguration());
      motors.add(new TalonFX(config.turnID()));
      motorConfigs.add(new TalonFXConfiguration());
    }

    SmartDashboard.putBoolean("Brake Mode", brakeModeStartsEnabled);
  }

  @Override
  public void robotPeriodic() {
    setBreakMode(SmartDashboard.getBoolean("Brake Mode", brakeModeStartsEnabled));
    SmartDashboard.putString(
        "Motor 0 Neutral Mode", motorConfigs.get(0).MotorOutput.NeutralMode.toString());
  }

  private void setBreakMode(boolean enabled) {
    if (enabled != brakeModeEnabled) {
      System.out.println("brake mode " + enabled);
      for (int i = 0; i < motors.size(); i++) {
        motorConfigs.get(i).MotorOutput.NeutralMode =
            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motors.get(i).getConfigurator().apply(motorConfigs.get(i));
      }
    }
    brakeModeEnabled = enabled;
  }
}
