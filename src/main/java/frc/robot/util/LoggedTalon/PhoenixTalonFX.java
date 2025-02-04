package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
// import frc.robot.util.LoggedTalon.Follower.TalonFXFollower;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class PhoenixTalonFX extends LoggedTalonFX {
  protected final TalonFX[] talonFX;
  private BaseStatusSignal[] statusSignals;
  private boolean statusSignalChanged = false;
  private final Debouncer[] connectionDebouncer;

  private final List<StatusSignal<Voltage>> voltageSignal;
  private final List<StatusSignal<Current>> torqueCurrentSignal;
  private final List<StatusSignal<Current>> supplyCurrentSignal;
  private final List<StatusSignal<AngularVelocity>> velocitySignal;
  private final List<StatusSignal<Angle>> positionSignal;

  public PhoenixTalonFX(int canID, CANBus canBus, String name, PhoenixTalonFollower... followers) {
    
    super(name,followers.length);
    
    talonFX = new TalonFX[followers.length+1];
    connectionDebouncer = new Debouncer[followers.length+1];
    voltageSignal = new ArrayList<>(followers.length+1);
    torqueCurrentSignal = new ArrayList<>(followers.length+1);
    supplyCurrentSignal = new ArrayList<>(followers.length+1);
    velocitySignal = new ArrayList<>(followers.length+1);
    positionSignal = new ArrayList<>(followers.length+1);

    for (int i = 0; i <= followers.length; i++) {
      if (i == 0) {
        talonFX[0] = new TalonFX(canID, canBus);
      } else {
        talonFX[i] = new TalonFX(followers[i-1].canid(),canBus);
      }
      connectionDebouncer[i] = new Debouncer(0.5);
      voltageSignal.set(i, talonFX[i].getMotorVoltage());
      torqueCurrentSignal.set(i, talonFX[i].getTorqueCurrent());
      supplyCurrentSignal.set(i, talonFX[i].getSupplyCurrent());
      velocitySignal.set(i,talonFX[i].getVelocity());
      positionSignal.set(i,talonFX[i].getPosition());
    }
  }

  @Override
  public void setControl(ControlRequest controlRequest) {
    talonFX[0].setControl(controlRequest);
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    for (int i = 1; i <= super.followers; i++) {
      
    }
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
  public void quickApplySlot0Config(Slot0Configs config) {
    PhoenixUtil.tryUntilOk(3, () -> talonFX[0].getConfigurator().apply(config));
  }
}
