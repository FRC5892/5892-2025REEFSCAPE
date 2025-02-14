package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;

public class SimpleMotorSim extends BaseTalonFXSim {
  private final DCMotorSim motorSim;

  public SimpleMotorSim(
      int canID,
      CANBus canBus,
      String name,
      double JKgMetersSquared,
      double gearReduction,
      PhoenixTalonFollower... followers) {
    super(canID, canBus, name, followers);
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(followers.length + 1), JKgMetersSquared, gearReduction),
            DCMotor.getKrakenX60Foc(followers.length + 1));
  }

  @Override
  protected void simulationPeriodic(TalonFXInputs inputs) {
    motorSim.setInputVoltage(motorSimState.getMotorVoltage());
    motorSim.update(0.02);
    motorSimState.setRotorVelocity(motorSim.getAngularVelocity());
    motorSimState.setRawRotorPosition(motorSim.getAngularPosition());
  }
}
