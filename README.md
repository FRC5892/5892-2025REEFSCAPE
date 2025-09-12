# Typhoon (FRC 5892) - 2025 Reefscape

Welcome to the controls and hardware reference for Typhoon, FRC team 5892's robot for the 2025 REEFSCAPE game
This document summarizes the main driver and codriver controls, as well as the CAN IDs for all major hardware components.  

## Controls

| Function                        | Controller | Button / Stick                         | Description                              |
|----------------------------------|------------|----------------------------------------|------------------------------------------|
| Drive (field-oriented)           | Driver     | Left Stick Y/X, Right Stick X (theta)  | Main driving control (rotation = right stick) |
| Reset Gyro                       | Driver     | Y Button                               | Sets robot's heading to zero             |
| Align to Reef (Left)             | Driver     | Left Bumper                            | Aligns robot to left reef                |
| Align to Reef (Right)            | Driver     | Right Bumper                           | Aligns robot to right reef               |
| Nudge Forward                    | Driver     | D-pad Up (POV Up)                      | Small forward movement                   |
| Nudge Backward                   | Driver     | D-pad Down (POV Down)                  | Small backward movement                  |
| Nudge Left                       | Driver     | D-pad Left (POV Left)                  | Small left movement                      |
| Nudge Right                      | Driver     | D-pad Right (POV Right)                | Small right movement                     |
| Elevator to Level 2 (L2)         | Codriver   | B Button                               | Moves elevator to Level 2                |
| Elevator to Level 3 (L3)         | Codriver   | X Button                               | Moves elevator to Level 3                |
| Elevator to Level 4 (L4)         | Codriver   | Y Button                               | Moves elevator to Level 4                |
| Elevator to Intake               | Codriver   | Right Bumper                           | Moves elevator to intake position        |
| Elevator Home                    | Codriver   | Start Button                           | Returns elevator to home                 |
| Coral Outtake                    | Codriver   | Left Bumper                            | Runs coral end effector out              |
| Funnel Up, Climb Extend, Tab Set | Codriver   | D-pad Up (POV Up)                      | Moves funnel up, extends climb, sets tab |
| Funnel Down, Tab Set             | Codriver   | D-pad Right (POV Right)                | Moves funnel down, sets tab              |
| Climb Retract                    | Codriver   | D-pad Down (POV Down)                  | Retracts climb                           |
| Funnel to Starting               | Codriver   | D-pad Left (POV Left)                  | Moves funnel to starting position        |

---

## CAN IDs

| Device              | CAN ID | Notes                                            |
|---------------------|--------|--------------------------------------------------|
| Elevator            | 20     |                                                  |
| Coral End Effector  | 22     |                                                  |
| Climb               | 23     |                                                  |
| Servo Hub           | 30     |                                                  |
| Front Left Drive    | 1      | Swerve module                                    |
| Front Left Steer    | 2      | Swerve module                                    |
| Front Right Drive   | 3      | Swerve module                                    |
| Front Right Steer   | 4      | Swerve module                                    |
| Back Left Drive     | 5      | Swerve module                                    |
| Back Left Steer     | 6      | Swerve module                                    |
| Back Right Drive    | 7      | Swerve module                                    |
| Back Right Steer    | 8      | Swerve module                                    |