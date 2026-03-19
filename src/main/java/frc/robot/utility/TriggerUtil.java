package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerUtil {
  private TriggerUtil() {}

  /**
   * Constantly starts the given command while the button is held.
   *
   * <p>Alternative to whileTrue that will restart the command even when the command ends
   *
   * <p>{@link Command#schedule()} will be called repeatedly while the trigger is active, and will
   * be canceled when the trigger becomes inactive.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public static void whileTrueContinuous(Trigger trigger, final Command command) {
    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            new Runnable() {
              private boolean m_pressedLast = trigger.getAsBoolean();

              @Override
              public void run() {
                boolean pressed = trigger.getAsBoolean();

                if (pressed) {
                  CommandScheduler.getInstance().schedule(command);
                } else if (m_pressedLast) {
                  command.cancel();
                }

                m_pressedLast = pressed;
              }
            });
  }
}
