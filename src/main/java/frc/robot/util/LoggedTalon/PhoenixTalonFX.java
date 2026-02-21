package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
// import frc.robot.util.LoggedTalon.Follower.TalonFXFollower;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.PhoenixUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class PhoenixTalonFX extends LoggedTalonFX {
  protected final TalonFX[] talonFX;
  private final Debouncer[] connectionDebouncer;

  private final List<StatusSignal<Voltage>> voltageSignal;
  private final List<StatusSignal<Current>> torqueCurrentSignal;
  private final List<StatusSignal<Current>> supplyCurrentSignal;
  private final List<StatusSignal<Temperature>> temperatureSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Angle> positionSignal;

  public PhoenixTalonFX(int canID, CANBus canBus, String name, PhoenixTalonFollower... followers) {

    super(name, followers.length);

    talonFX = new TalonFX[followers.length + 1];
    connectionDebouncer = new Debouncer[followers.length + 1];
    voltageSignal = new ArrayList<>();
    torqueCurrentSignal = new ArrayList<>();
    supplyCurrentSignal = new ArrayList<>();
    temperatureSignal = new ArrayList<>();
    Follower follower = new Follower(canID, MotorAlignmentValue.Aligned);
    for (int i = 0; i <= followers.length; i++) {
      if (i == 0) {
        talonFX[0] = new TalonFX(canID, canBus);
      } else {
        talonFX[i] = new TalonFX(followers[i - 1].canid(), canBus);
        talonFX[i].setControl(follower.withMotorAlignment(followers[i - 1].opposeDirection()));
      }
      connectionDebouncer[i] = new Debouncer(0.5);
      voltageSignal.add(i, talonFX[i].getMotorVoltage());
      torqueCurrentSignal.add(i, talonFX[i].getTorqueCurrent());
      supplyCurrentSignal.add(i, talonFX[i].getSupplyCurrent());
      temperatureSignal.add(i, talonFX[i].getDeviceTemp());
    }
    velocitySignal = talonFX[0].getVelocity();
    positionSignal = talonFX[0].getPosition();
  }

  @Override
  public void setControl(ControlRequest controlRequest) {
    talonFX[0].setControl(controlRequest);
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    for (int i = 0; i <= super.followers; i++) {
      final StatusCode status;
      if (i == 0) {
        status =
            StatusSignal.refreshAll(
                voltageSignal.get(0),
                torqueCurrentSignal.get(0),
                supplyCurrentSignal.get(0),
                temperatureSignal.get(0),
                velocitySignal,
                positionSignal);
      } else {
        status =
            StatusSignal.refreshAll(
                voltageSignal.get(i),
                torqueCurrentSignal.get(i),
                supplyCurrentSignal.get(i),
                temperatureSignal.get(i));
      }
      inputs.connected[i] = connectionDebouncer[i].calculate(status.isOK());
      inputs.appliedVolts[i] = voltageSignal.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentSignal.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentSignal.get(i).getValueAsDouble();
      inputs.temperatureC[i] = temperatureSignal.get(i).getValueAsDouble();
    }
    inputs.positionRot = positionSignal.getValueAsDouble();
    inputs.velocityRotPS = velocitySignal.getValueAsDouble();
  }

  // This is when I wish java had macro support

  @Override
  public LoggedTalonFX withConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(5, () -> talonFX[0].getConfigurator().apply(config));
    return this;
  }

  @Override
  public LoggedTalonFX withSimConfig(Function<TalonFXConfiguration, TalonFXConfiguration> config) {
    return this;
  }

  @Override
  public void quickApplyConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(3, () -> talonFX[0].getConfigurator().apply(config));
  }

  @Override
  public void setPosition(Angle position) {
    talonFX[0].setPosition(position);
  }
}
