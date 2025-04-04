// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.util.LoggedTunableMeasure;
import java.util.function.Supplier;

public class ElevatorConstants {
  public static final double GEAR_RATIO =
      6.1252998684746585; // Don't ask, it works, at least for 15:1 on max planetary

  // Max planetary 5:1, 3:1, 54:42, 22 teeth chain pitch is 1/4 in
  // Overall: 32.143:1
  // 64.286:1
  // 5.5 in per rotation
  // 2:1 elevator gearing
  public static final Distance DISTANCE_PER_ROTATION = Inches.of(5.5);
  public static final double VELOCITY_TOLERANCE_RPM = 0.1;

  public enum ElevatorPosition {
    INTAKE(new LoggedTunableMeasure<>("Elevator/Intake", Meters.mutable(0.0))),
    L2(new LoggedTunableMeasure<>("Elevator/L2", Meters.mutable(0.29))),
    L3(new LoggedTunableMeasure<>("Elevator/L3", Meters.mutable(0.69))),
    L4(new LoggedTunableMeasure<>("Elevator/L4", Meters.mutable(1.265)));

    public final Supplier<MutDistance> height;

    ElevatorPosition(Supplier<MutDistance> heightMeters) {
      this.height = heightMeters;
    }
  }
}
