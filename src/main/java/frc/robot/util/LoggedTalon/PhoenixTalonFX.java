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
// import frc.robot.util.LoggedTalon.Follower.TalonFXFollower;
import frc.robot.util.PhoenixUtil;
import java.util.ArrayList;
import java.util.function.Function;

public class PhoenixTalonFX extends LoggedTalonFX {
  protected final TalonFX talonFX;
  private BaseStatusSignal[] statusSignals;
  private boolean statusSignalChanged = false;
  private final Debouncer connectionDebouncer = new Debouncer(0.5);

  // private final TalonFX[] followers;

  protected PhoenixTalonFX(int canID, CANBus canBus, String name, int followers) {
    super(name, followers);
    // this.followers = null;
    talonFX = new TalonFX(canID, canBus);
  }

  public PhoenixTalonFX(int canID, CANBus canBus, String name /*, TalonFXFollower... followers*/) {
    super(name, 0);
    // if (followers.length != 0) {
    //   for (TalonFX follower : followers) {
    //   }
    //   Arrays.stream(followers).map(talonFXFollower -> talonFXFollower.).toArray(Object[]::new);
    // } else {
    //     this.followers = null;
    // }
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
  public LoggedTalonFX withAppliedVoltage() {
    statusSignalChanged = true;
    voltageSignal = talonFX.getMotorVoltage();
    return this;
  }

  private StatusSignal<Current> torqueCurrentSignal = null;

  @Override
  public LoggedTalonFX withTorqueCurrent() {
    statusSignalChanged = true;
    torqueCurrentSignal = talonFX.getTorqueCurrent();
    return this;
  }

  private StatusSignal<Current> statorCurrentSignal = null;

  @Override
  public LoggedTalonFX withStatorCurrent() {
    statusSignalChanged = true;
    statorCurrentSignal = talonFX.getStatorCurrent();
    return this;
  }

  private StatusSignal<AngularVelocity> velocitySignal = null;

  @Override
  public LoggedTalonFX withVelocity() {
    statusSignalChanged = true;
    velocitySignal = talonFX.getVelocity();
    return this;
  }

  private StatusSignal<Angle> positionSignal = null;

  @Override
  public LoggedTalonFX withPosition() {
    statusSignalChanged = true;
    positionSignal = talonFX.getPosition();
    return this;
  }

  @Override
  public LoggedTalonFX withConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
    return this;
  }

  @Override
  public LoggedTalonFX withSimConfig(Function<TalonFXConfiguration, TalonFXConfiguration> config) {
    return this;
  }

  @Override
  public void quickApplySlot0Config(Slot0Configs config) {
    PhoenixUtil.tryUntilOk(3, () -> talonFX.getConfigurator().apply(config));
  }
}
