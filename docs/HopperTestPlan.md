# Hopper Subsystem Test Plan
As a reference:
- *Hopper* refers to the combined subsystem, and physical robot storage area/mechanisms.
- *Bubbler* refers to the belt component that pushes fuel towards the back of the robot for firing.
- *Feeder* refers to the wheel system that pushes fuel up into the shooter.

## Preparation
1. Pull the latest code from the `jim-hopper` branch (note: change this to `main` once merged).
2. Set the proper constants in `HopperConstants`.java:
    - Set `BUBBLER_CAN_ID` to the bubbler motor's CAN ID (make sure the appropriate `switch`/`case` label is used for the robot type).
    - Set `FEEDER_CAN_ID` to the feeder motor's CAN ID (make sure the appropriate `switch`/`case` label is used for the robot type).
    - Set `BUBBLER_PID` to the appropriate PID constants for the bubbler.
    - Set `FEEDER_PID` to the appropriate PID constants for the feeder.
    - Set `BUBBLER_FF` to the appropriate feedforward constants for the bubbler.
    - Set `FEEDER_FF` to the appropriate feedforward constants for the feeder.
    - Check that `BUBBLER_GEAR_RATIO` and `FEEDER_GEAR_RATIO` match the actual gear ratios of the motors.
3. Check that `RUNNING_TEST_PLANS` in `Constants.java` is set to `true`.

## Setup
1. Put the battery in the robot and check that everything is connected properly.
2. Open the Driver Station and connect to the robot (**DO NOT ENABLE**).
3. Connect an Xbox controller and assign it as the operator controller (USB icon tab, drag controller name to the `1` row).
4. Deploy code to robot (In VS Code: `Ctrl+Shift+P` -> `WPILib: Deploy Robot Code`).
5. Enable robot in **Tele-Op** mode.
6. Ensure that the bubbler and feeder are **not spinning**. If they are, **STOP!**

## Execution
1. Press and hold the B button on the operator controller for 3 seconds, then release.
    - Bubbler should start spinning at a low speed, with the belts moving towards the shooter.
        - **If the belts are spinning the wrong way: release B, disable robot, and invert `RunMode` bubbler speeds in `HopperConstants.java`.**
    - Feeder motor should not spin.
    - Bubbler should spin while the button is held, then stop spinning when released.
2. Press and hold down the right trigger on the operator controller for 3 seconds, then release.
    - Bubbler should start spinning at a high speed, with the belts moving towards the shooter.
    - Feeder wheels should start spinning such that they spin towards the front of the robot.
        - **If the wheels are spinning the wrong way: release the trigger, disable robot, and invert `RunMode` feeder speeds in `HopperConstants.java`.**
    - Bubbler and feeder should spin while the button is held, then stop spinning when released.
3. Press and hold the start button (the small one with three lines) on the operator controller for 3 seconds, then release.
    - Bubbler should start spinning with the belts moving away from shooter.
    - Feeder wheels should start spinning such that they spin towards the back of the robot.
    - **If the motors are spinning the wrong way: release start, disable robot, and check that the `RunMode.Reverse` feeder speeds in `HopperConstants.java` are inverted from the regular modes.**
    - Bubbler and feeder should spin while the button is held, then stop spinning when released.
