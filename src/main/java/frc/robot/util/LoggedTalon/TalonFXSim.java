package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.Function;

public class TalonFXSim extends PhoenixTalonFX {
  protected TalonFXConfiguration config = new TalonFXConfiguration();
  protected final TalonFXSimState motorSimState;
  protected DCMotorSim physicsSim;
  protected final DCMotor motorType;
  protected final double inertia;

  public TalonFXSim(
      int CAN_ID, CANBus canBus, String name, boolean reverse, DCMotor motorType, double inertia) {
    super(CAN_ID, canBus, name);
    this.motorType = motorType;
    this.inertia = inertia;
    motorSimState = super.talonFX.getSimState();
    if (reverse) {
      motorSimState.Orientation = ChassisReference.Clockwise_Positive;
    }
  }

  @Override
  public LoggedTalonFX withConfig(TalonFXConfiguration config) {
    this.config = config;
    super.withConfig(config);
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motorType, inertia, config.Feedback.SensorToMechanismRatio),
            DCMotor.getKrakenX60(1),
            0.5,
            0.5);
    return this;
  }

  @Override
  public LoggedTalonFX withSimConfig(
      Function<TalonFXConfiguration, TalonFXConfiguration> configFunction) {
    withConfig(configFunction.apply(this.config));
    return this;
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    if (physicsSim == null) {
      throw new UnsupportedOperationException(
          "TalonFXSim must have a config before a periodic cycle!");
    }
    super.updateInputs(inputs);
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(motorSimState.getMotorVoltage());
    physicsSim.update(0.02);

    //    motorSimState.setRawRotorPosition(physicsSim.getAngularPosition());
    //    motorSimState.setRotorVelocity(physicsSim.getAngularVelocity());
  }
}
