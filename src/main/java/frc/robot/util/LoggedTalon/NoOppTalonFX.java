package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import java.util.function.Function;

public class NoOppTalonFX extends LoggedTalonFX {

  public NoOppTalonFX(String name, int followers) {
    super(name, followers);
  }

  @Override
  public void setControl(ControlRequest controlRequest) {}

  @Override
  protected void updateInputs(TalonFXInputs inputs) {}

  @Override
  public LoggedTalonFX withConfig(TalonFXConfiguration config) {
    return this;
  }

  @Override
  public LoggedTalonFX withSimConfig(Function<TalonFXConfiguration, TalonFXConfiguration> config) {
    return this;
  }

  @Override
  public void quickApplySlot0Config(Slot0Configs config) {}
}
