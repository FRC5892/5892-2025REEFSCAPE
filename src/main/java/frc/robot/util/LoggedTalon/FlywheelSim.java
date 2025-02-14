package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;

public class FlywheelSim extends BaseTalonFXSim {

  public FlywheelSim(int canID, CANBus canBus, String name, PhoenixTalonFollower... followers) {
    super(canID, canBus, name, followers);
  }

  @Override
  protected void simulationPeriodic(TalonFXInputs inputs) {}
}
