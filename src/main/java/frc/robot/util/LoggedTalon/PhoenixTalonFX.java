package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

import java.util.ArrayList;

public class PhoenixTalonFX extends LoggedTalonFX {
  protected final TalonFX talonFX;
  private BaseStatusSignal[] statusSignals;
  private boolean statusSignalChanged = false;
  private final Debouncer connectionDebouncer = new Debouncer(0.5);

  public PhoenixTalonFX(int canID, CANBus canBus, String name, ImplementedSignals... signals) {
    super(name);
    talonFX = new TalonFX(canID, canBus);
  }

  @Override
  public void setControl(ControlRequest controlRequest) {
    talonFX.setControl(controlRequest);
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    if (statusSignalChanged) {
      ArrayList<BaseStatusSignal> statusSignalsList = new ArrayList<>();
      if (voltageSignal != null) {
        statusSignalsList.add(voltageSignal);
      }
      if (torqueCurrentSignal != null) {
        statusSignalsList.add(torqueCurrentSignal);
      }
      if (statorCurrentSignal != null) {
        statusSignalsList.add(statorCurrentSignal);
      }
      if (velocitySignal != null) {
        statusSignalsList.add(velocitySignal);
      }
      if (positionSignal != null) {
        statusSignalsList.add(positionSignal);
      }
      this.statusSignals = statusSignalsList.toArray(new BaseStatusSignal[0]);
      statusSignalChanged = false;
    }
    StatusCode status = BaseStatusSignal.refreshAll(statusSignals);
    inputs.connected = connectionDebouncer.calculate(status == StatusCode.OK);
    if (voltageSignal != null) {
      inputs.appliedVoltage = voltageSignal.getValue();
    }
    if (torqueCurrentSignal != null) {
      inputs.torqueCurrent = torqueCurrentSignal.getValue();
    }
    if (statorCurrentSignal != null) {
      inputs.statorCurrent = statorCurrentSignal.getValue();
    }
    if (velocitySignal != null) {
      inputs.velocity = velocitySignal.getValue();
    }
    if (positionSignal != null) {
      inputs.position = positionSignal.getValue();
    }
  }
  // This is when I wish java had macro support
  private StatusSignal<Voltage> voltageSignal = null;

  @Override
  public void withAppliedVoltage() {
    statusSignalChanged = true;
    voltageSignal = talonFX.getMotorVoltage();
  }

  private StatusSignal<Current> torqueCurrentSignal = null;

  @Override
  public void withTorqueCurrent() {
    statusSignalChanged = true;
    torqueCurrentSignal = talonFX.getTorqueCurrent();
  }

  private StatusSignal<Current> statorCurrentSignal = null;

  @Override
  public void withStatorCurrent() {
    statusSignalChanged = true;
    statorCurrentSignal = talonFX.getStatorCurrent();
  }

  private StatusSignal<AngularVelocity> velocitySignal = null;

  @Override
  public void withVelocity() {
    statusSignalChanged = true;
    velocitySignal = talonFX.getVelocity();
  }

  private StatusSignal<Angle> positionSignal = null;

  @Override
  public void withPosition() {
    statusSignalChanged = true;
    positionSignal = talonFX.getPosition();
  }

  @Override
  public void applyConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
  }
  @Override
  public void quickApplySlot0Config(Slot0Configs config) {
    PhoenixUtil.tryUntilOk(3, () -> talonFX.getConfigurator().apply(config));
  }
}
