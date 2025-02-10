package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTalon.NoOppTalonFX;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorTest {

  private Elevator elevator;

  @BeforeEach
  void setUp() {
    elevator = new Elevator(new NoOppTalonFX("Elevator", 0));
  }

  @Test
  void convertDistanceAngleDistance() {
    Distance[] distances =
        new Distance[] {
          Meters.of(0), Meters.of(Math.PI), Meters.of(2), Meters.of(1), Inches.of(10), Inches.of(17)
        };
    for (Distance distance : distances) {
      Angle angle = Elevator.distanceToAngle(distance);
      Distance convertedDistance = Elevator.angleToDistance(angle);
      assertEquals(distance, convertedDistance);
    }
  }

  @Test
  void convertAngleDistanceAngle() {
    Angle[] angles =
        new Angle[] {
          Rotations.of(12), Degrees.of(90), Degrees.of(180), Degrees.of(270), Degrees.of(360)
        };
    for (Angle angle : angles) {
      Distance distance = Elevator.angleToDistance(angle);
      Angle convertedAngle = Elevator.distanceToAngle(distance);
      assertEquals(angle, convertedAngle);
    }
  }
}
