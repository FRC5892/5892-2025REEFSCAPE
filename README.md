# Typhoon (FRC 5892) - 2025 Reefscape

Welcome to the repository for Typhoon, FRC Team 5892 Energy HEROs robot for the 2025 REEFSCAPE season

## About Team 5892
Team 5892, the Energy HEROs, is an FRC team from Energy Institute High School in Houston, Texas, founded in 2016.

## About Typhoon (2025)
Typhoon is our robot entry for the 2025 FRC competition, REEFSCAPE. Typhoon features a swerve drive chain (a system allowing wheels to rotate and move in any direction), a coral source intake with a retractable funnel (an intake mechanism that collects coral-shaped game pieces using a funnel that can extend or retract), and is able to score on L2-L4 corals (referring to the second through fourth scoring levels for these game pieces) and climb the deep cage (the elevated structure used for endgame climbing) during the endgame. During the season, we ranked 26th in the Texas district with 186 total points scored and a 30-19-0 record in official gameplay. 

## Controls

| Function           | Controller | Button / Stick           | Description            |
|--------------------|------------|--------------------------|------------------------|
| Drive (field-oriented) | Driver     | Left Stick (Y/X)         | Robot translation      |
| Drive (field-oriented) | Driver     | Right Stick (X) (theta)  | Robot rotation         |
| Reset Gyro         | Driver     | Y Button                 | Sets robot's heading to zero             |
| Align to Reef (Left)  | Driver     | Left Bumper              | Aligns robot to left reef                |
| Align to Reef (Right) | Driver     | Right Bumper             | Aligns robot to right reef               |
| Nudge Forward      | Driver     | D-pad Up (POV Up)        | Small forward movement                   |
| Nudge Backward     | Driver     | D-pad Down (POV Down)    | Small backward movement                  |
| Nudge Left         | Driver     | D-pad Left (POV Left)    | Small left movement                      |
| Nudge Right        | Driver     | D-pad Right (POV Right)  | Small right movement                     |
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