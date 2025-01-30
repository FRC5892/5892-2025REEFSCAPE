// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
  // TODO: Tune Gear Ratio
  public static final double GEAR_RATIO = 55;
  public static final Distance DISTANCE_PER_ROTATION = Inches.of(11);

  public enum ElevatorTarget {
    INTAKE(Meters.of(0.0)),
    L2(Meters.of(0.5)),
    L3(Meters.of(1)),
    L4(Meters.of(2));

    public final Distance height;

    ElevatorTarget(Distance heightMeters) {
      this.height = heightMeters;
    }
  }
}
