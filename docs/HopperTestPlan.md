# Hopper Subsystem Test Plan
As a reference:
- *Hopper* refers to the combined subsystem, and physical robot storage area/mechanisms.
- *Feeder* refers to the belt component that pushes fuel towards the back of the robot for firing.
- *Lifter* refers to the wheel system that pushes fuel up into the shooter.

## Preparation
1. Pull the latest code from the `main` branch.
2. Set the proper constants in `HopperConstants`.java:
    - Set `FEEDER_CAN_ID` to the feeder motor's CAN ID (check that you're setting them for the correct `RobotType` in the `switch` statement).
    - Set `LIFTER_CAN_ID` to the lifter motor's CAN ID (check that you're setting them for the correct `RobotType` in the `switch` statement).
    - Set `FEEDER_PID` to the appropriate PID constants for the feeder.
    - Set `LIFTER_PID` to the appropriate PID constants for the lifter.
    - Set `FEEDER_FF` to the appropriate feedforward constants for the feeder.
    - Set `LIFTER_FF` to the appropriate feedforward constants for the lifter.
    - Check that `FEEDER_GEAR_RATIO` and `LIFTER_GEAR_RATIO` match the actual gear ratios of the motors.
    - Check that the speeeds in the `RunMode` enum are correct (e.g., reverse speed is inverted from the others, speeds match the speed we want to run at, etc.)
        - The feeder speed is the first parameter, the lifter speed is the second parameter
3. Enable development mode (revealing test autos) by setting `DEVELOPMENT_MODE` to `true` in `Constants.java`

## Setup
1. Put the battery in the robot and check that everything is connected properly.
2. Open the Driver Station and connect to the robot (**DO NOT ENABLE**).
3. Open Elastic and make sure that it is properly connected to the robot (check the IP at the bottom left).
4. Deploy code to robot (In VS Code: `Ctrl+Shift+P` -> `WPILib: Deploy Robot Code`).
5. Select the `[Test] Hopper Test Routine` mode in the Elastic Auto Chooser.

## Execution
> [!CAUTION]
> ### 🛑 STOP! Did You Really Read The Instructions? 🛑
> As a reminder, excise your best judgement while testing.
> **If anything seems off, or the robot starts making weird noises, or something breaks, DISABLE THE ROBOT IMMEDIATELY.**

1. Enable the robot in **Autonomous** mode.
2. Fuel Storage Mode Test
    - Feeder should start spinning at a low speed for **5 seconds** and then stop, with the belts moving towards the shooter.
        - **If the belts are spinning the wrong way: disable robot, and invert `RunMode` feeder speeds in `HopperConstants.java`.**
    - Lifter motor should not spin.
3. All hopper motors should remain still for **2 seconds**.
4. Firing Mode Test
    - Feeder should start spinning at a high speed, with the belts moving towards the shooter.
    - Lifter wheels should start spinning such that they spin towards the front of the robot.
        - **If the wheels are spinning the wrong way: disable robot, and invert `RunMode` lifter speeds in `HopperConstants.java`.**
    - Feeder and lifter should spin for **5 seconds** and then stop.
4. All hopper motors should remain still for **2 seconds**.
5. Reverse Mode Test
    - Feeder should start spinning with the belts moving away from shooter.
    - Lifter wheels should start spinning such that they spin towards the back of the robot.
    - **If the motors are spinning the wrong way: disable robot, and check that the `RunMode.Reverse` feeder and lifter speeds in `HopperConstants.java` are inverted from the regular modes.**
    - Feeder and lifter should spin for **5 seconds** and then stop.
6. Disable the robot.
