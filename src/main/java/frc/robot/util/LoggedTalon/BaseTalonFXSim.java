package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import java.util.function.Function;

public abstract class BaseTalonFXSim extends PhoenixTalonFX {
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  protected final TalonFXSimState motorSimState;

  public BaseTalonFXSim(int canID, CANBus canBus, String name, PhoenixTalonFollower... followers) {
    super(canID, canBus, name, followers);
    motorSimState = super.talonFX[0].getSimState();
  }

  @Override
  public LoggedTalonFX withSimConfig(
      Function<TalonFXConfiguration, TalonFXConfiguration> configFunction) {
    withConfig(configFunction.apply(this.config));
    return this;
  }

  @Override
  public LoggedTalonFX withConfig(TalonFXConfiguration config) {
    this.config = config;
    super.withConfig(config);
    afterConfigApplied(config);
    return this;
  }

  public void afterConfigApplied(TalonFXConfiguration config) {}

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simulationPeriodic(inputs);
    super.updateInputs(inputs);
  }

  protected abstract void simulationPeriodic(TalonFXInputs inputs);
}
