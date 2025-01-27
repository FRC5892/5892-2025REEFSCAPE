package frc.robot.subsystems.Elevator;

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
  private final double kElevatorDrumRadius = 0.005;
  private final double kMinElevatorHeightMeters = 0;
  private final double kMaxElevatorHeightMeters = 2;

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          kMinElevatorHeightMeters,
          kMaxElevatorHeightMeters,
          true,
          0,
          0.0005,
          0.0);

  public ElevatorSimulation(int CAN_ID, CANBus canBus, String name) {
    super(CAN_ID, canBus, name);
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    elevatorSim.update(Robot.defaultPeriodSecs);
    double rawRotorPosition =
        elevatorSim.getPositionMeters() / (Math.PI * 2 * kElevatorDrumRadius * kElevatorGearing);
    super.motorSimState.setRawRotorPosition(rawRotorPosition);
    super.motorSimState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond()
            / (Math.PI * 2 * kElevatorDrumRadius)
            * kElevatorGearing);
    super.updateInputs(inputs);
    elevatorSim.setInputVoltage(super.motorSimState.getMotorVoltage());
    Logger.recordOutput("Elevator/ElevatorRawPosition", elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/ElevatorMotorPosition", rawRotorPosition);
  }

  @Override
  protected void simulationPeriodic(TalonFXInputs inputs) {}
}
