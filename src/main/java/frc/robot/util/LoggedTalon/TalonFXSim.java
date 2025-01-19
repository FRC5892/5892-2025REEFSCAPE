package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;

public class TalonFXSim extends PhoenixTalonFX {
  private final TalonFXSimState motorSimState;
  private final DCMotorSim physicsSim;

  public TalonFXSim(
      int CAN_ID,
      CANBus canBus,
      TalonFXConfiguration config,
      boolean reverse,
      String name,
      DCMotor motorType,
      double inertia) {
    super(CAN_ID, canBus, name);
    motorSimState = super.talonFX.getSimState();
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motorType, inertia, config.Feedback.SensorToMechanismRatio),
            motorType,
            0.5,
            0.5);
    if (reverse) {
      motorSimState.Orientation = ChassisReference.Clockwise_Positive;
    }
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    super.updateInputs(inputs);
//     motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage().baseUnitMagnitude());
            physicsSim.setInputVoltage(motorSimState.getMotorVoltage());
            physicsSim.update(0.02);

            motorSimState.setRawRotorPosition(physicsSim.getAngularPosition());
            motorSimState.setRotorVelocity(physicsSim.getAngularVelocity());
  }
}
