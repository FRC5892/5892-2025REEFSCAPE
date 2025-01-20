package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

public class NoOppTalonFX extends LoggedTalonFX {
  public NoOppTalonFX(String name) {
    super(name);
  }

  @Override
  public void setControl(ControlRequest controlRequest) {}

  @Override
  protected void updateInputs(TalonFXInputs inputs) {}

  @Override
  public LoggedTalonFX withAppliedVoltage() {
    return this;
  }

  @Override
  public LoggedTalonFX withTorqueCurrent() {
    return this;
  }

  @Override
  public LoggedTalonFX withStatorCurrent() {
    return this;
  }

  @Override
  public LoggedTalonFX withVelocity() {
    return this;
  }

  @Override
  public LoggedTalonFX withPosition() {
    return this;
  }

  @Override
  public void applyConfig(TalonFXConfiguration config) {}

  @Override
  public void quickApplySlot0Config(Slot0Configs config) {}
}
