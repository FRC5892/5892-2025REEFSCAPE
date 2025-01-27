// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class ElevatorConstants {

  public enum ElevatorTarget {
    INTAKE(0.0),
    L2(0.5),
    L3(0.7),
    L4(2);

    public final double height;

    ElevatorTarget(double heightMeters) {
      this.height = heightMeters;
    }
  }
}
