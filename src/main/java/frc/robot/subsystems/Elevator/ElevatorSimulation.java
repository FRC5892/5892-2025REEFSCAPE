package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import frc.robot.util.LoggedTalon.BaseTalonFXSim;
import frc.robot.util.LoggedTalon.TalonFXInputs;
import org.littletonrobotics.junction.Logger;

public class ElevatorSimulation extends BaseTalonFXSim {
  private final double kElevatorGearing = 3;
  private final double kCarriageMass = 5;
  private final double kMinElevatorHeightMeters = 0;
  private final double kMaxElevatorHeightMeters = 2;

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          kElevatorGearing,
          kCarriageMass,
          //          ElevatorConstants.DISTANCE_PER_ROTATION.in(Meters) / (2 * Math.PI),
          0.001,
          kMinElevatorHeightMeters,
          kMaxElevatorHeightMeters,
          true,
          0,
          0.00005,
          0.0);

  public ElevatorSimulation(int CAN_ID, CANBus canBus, String name) {
    super(CAN_ID, canBus, name);
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    elevatorSim.update(Robot.defaultPeriodSecs);

    double rawRotorPositionRotations =
        Elevator.distanceToAngle(Meters.of(elevatorSim.getPositionMeters())).in(Rotations)
            * ElevatorConstants.GEAR_RATIO;
    super.motorSimState.setRawRotorPosition(rawRotorPositionRotations);

    double rawRotorVelocityRotationsPerSec =
        Elevator.distanceToAngle(Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Rotations)
            * ElevatorConstants.GEAR_RATIO;
    super.motorSimState.setRotorVelocity(rawRotorVelocityRotationsPerSec);

    super.updateInputs(inputs);
    elevatorSim.setInputVoltage(super.motorSimState.getMotorVoltage());
    Logger.recordOutput("Elevator/ElevatorRawPosition", elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/ElevatorMotorPosition", rawRotorPositionRotations);
  }

  @Override
  protected void simulationPeriodic(TalonFXInputs inputs) {}
}
