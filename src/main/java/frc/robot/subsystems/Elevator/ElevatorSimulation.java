package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import frc.robot.util.LoggedTalon.BaseTalonFXSim;
import frc.robot.util.LoggedTalon.TalonFXInputs;
import org.littletonrobotics.junction.Logger;

public class ElevatorSimulation extends BaseTalonFXSim {
  private final double kCarriageMass = 5;
  private final double kMinElevatorHeightMeters = 0;
  private final double kMaxElevatorHeightMeters = 1.8;

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          ElevatorConstants.GEAR_RATIO,
          kCarriageMass,
          ElevatorConstants.DISTANCE_PER_ROTATION.in(Meters) / (2 * Math.PI),
          //          0.001,
          kMinElevatorHeightMeters,
          kMaxElevatorHeightMeters,
          true,
          0,
          0,
          0);

  public ElevatorSimulation(int CAN_ID, CANBus canBus, String name) {
    super(CAN_ID, canBus, name);
    motorSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  protected void simulationPeriodic(TalonFXInputs inputs) {
    elevatorSim.update(Robot.defaultPeriodSecs);

    double rawRotorPositionRotations =
        Elevator.distanceToAngle(Meters.of(elevatorSim.getPositionMeters())).in(Rotations)
            * ElevatorConstants.GEAR_RATIO;
    super.motorSimState.setRawRotorPosition(rawRotorPositionRotations);

    double rawRotorVelocityRotationsPerSec =
        Elevator.distanceToAngle(Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Rotations)
            * ElevatorConstants.GEAR_RATIO;
    super.motorSimState.setRotorVelocity(rawRotorVelocityRotationsPerSec);
    Logger.recordOutput("Elevator/ElevatorRawPosition", elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/ElevatorMotorPosition", rawRotorPositionRotations);
  }
}
