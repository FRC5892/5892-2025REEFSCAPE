// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableMeasure;
import java.util.function.Supplier;

public class ElevatorConstants {
  // TODO: Tune Gear Ratio
  public static final double GEAR_RATIO = 55;
  public static final Distance DISTANCE_PER_ROTATION = Inches.of(11);
  public static final double DISTANCE_TOLERANCE_METERS = 0.01;
  public static final double VELOCITY_TOLERANCE_RPM = 0.1;

  public enum ElevatorPosition {
    INTAKE(new LoggedTunableMeasure<>("Elevator/Intake", Meters.of(0.0))),
    L2(new LoggedTunableMeasure<>("Elevator/L2", Meters.of(0.5))),
    L3(new LoggedTunableMeasure<>("Elevator/L3", Meters.of(1))),
    L4(new LoggedTunableMeasure<>("Elevator/L4", Meters.of(2)));

    public final Supplier<Distance> height;

    ElevatorPosition(Supplier<Distance> heightMeters) {
      this.height = heightMeters;
    }
  }
}
