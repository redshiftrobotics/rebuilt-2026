# Intake Test Plan

## Preparation
1. Pull code from `Intake-Slapdown` branch or `Main` if it has been merged
2. Fill in all constants in `IntakeConstants.java`:
    - `INTAKE_WHEEL_SPEED` should be the wheel motor's CAN ID.
    - `SLAPDOWN_DOWN_SETPOINT` should be set to the correct angle for the slapdown mechanism to lower
    - `SLAPDOWN_UP_SETPOINT` should be set to the correct angle for the slapdown mechanism to raise

    - `SLAPDOWN_PID` should be tuned to the appropriate values.

    - `INTAKE_WHEEL_CAN_ID` should be the wheel motor's CAN ID.
    - `SLAPDOWN_CAN_ID` should be the slapdown motor's CAN ID.

    - `WHEEL_GEAR_RATIO` should be the wheel motor's gear ratio.
    - `SLAPDOWN_GEAR_RATIO` should be the slapdown motor's gear ratio.

## Setup
1. Make sure the slapdown mechanism is in the up postition

## Execution
1. Press the left trigger on the operator controller
    - The slapdown should drop
    - And the intake wheels should spin
2. Release the left trigger on the operator controller
    - The slapdown should raise
    - And the intake wheels should stop