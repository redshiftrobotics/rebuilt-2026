# Intake Test Plan

## Preparation
1. Pull code from `Intake-Slapdown` branch or `Main` if it has been merged
2. Fill in all constants in `IntakeConstants.java`:
    - `INTAKE_WHEEL_SPEED` should be the wheel motor's CAN ID.
    - `SLAPDOWN_DOWN_SETPOINT` should be set to the correct angle for the slapdown mechanism to lower to

    - `SLAPDOWN_PID` should be tuned to the appropriate values.

    - `WHEEL_CAN_ID` should be the wheel motor's CAN ID.
    - `SLAPDOWN_CAN_ID` should be the slapdown motor's CAN ID.

    - `WHEEL_GEAR_RATIO` should be the wheel motor's gear ratio.
    - `SLAPDOWN_GEAR_RATIO` should be the slapdown motor's gear ratio.

## Setup
1. Make sure the slapdown mechanism is up

## Execution
1. Press the left bumper on the operator controller
    - The slapdown mechanism should lower to the desired angle
2. Press the left trigger on the operator controller
    - The compliant wheel mechanism should begin to spin
3. Press the left trigger on the operator controller again
    - The compliant wheel mechanism should toggle off
