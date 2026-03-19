package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Tests for RobotContainer */
public class RobotContainerTest {

  @Test
  @DisplayName("Instantiate RobotContainer")
  public void testRobotContainer() {
    assertDoesNotThrow(RobotContainer::new);
  }
}
