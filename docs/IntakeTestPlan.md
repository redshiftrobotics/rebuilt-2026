# Intake Test Plan

## Preparation
1. Pull code from `Intake-Slapdown` branch
1. Fill in all constants in `IntakeConstants.java`:
    - `PID_CONFIG` should be tuned to the appropriate values.
    - `WHEEL_CAN_ID` should be the wheel motor's CAN ID.
    - `SLAPDOWN_CAN_ID` should be the slapdown motor's CAN ID.
    - `WHEEL_GEAR_ID` should be the wheel motor's gear ratio.
    - `SLAPDOWN_GEAR_RATIO` should be the slapdown motor's gear ratio.
    - `SLAPDOWN_DOWN_SETPOINT` should be set to the correct angle for the slapdown mechanism to lower to

## Setup
1. Make sure that the battery is in
2. Make sure the slapdown mechanism is up
3. Connect to robot in driver station
4. Connect and assign an XBOX controller to operator controller
5. Enable Tele-op mode in the drivers station

## Execution
1. Press the left bumper on the controller
    - The slapdown mechanism should lower to the desired angle
2. Press the left trigger on the controller
    - The compliant wheel mechanism should begin to spin
3. Press the left trigger on the controller again
    - The compliant wheel mechanism should toggle off
