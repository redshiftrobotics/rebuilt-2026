> [!NOTE]  
> This is a template test plan. While I ask that you keep the general format & headings, make sure to adjust it to your needs.
> Test plans exist to verify that your code's *expected* and **actual** behaviours match up. They should have three phases:
> 1. **Preparation:** These steps should all be completed *before* the robot is even put down to be tested.
> 2. **Setup:** These steps should be completed once the robot is on the ground.
> 3. **Execution:** Once 

## Preparation
1. Fill in all constants in TemplateConstants.java:
    - `PID_CONFIG` should be tuned to the appropriate values.
    - `MOTOR_ID` should be the motor's CAN ID.
2. *etc, etc*.

## Setup
1. Make sure that the battery is in
2. Put the mechanism in the right position.

## Execution
1. Set `SHOW_TEST_AUTOS` to `true`.
3. Push code and select the auto "[TEST] My Test Plan"
4. Enable in auto. *The mechanism should shoot fuel.*