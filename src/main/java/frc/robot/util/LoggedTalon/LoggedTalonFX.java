package frc.robot.util.LoggedTalon;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants;
// import frc.robot.util.LoggedTalon.Follower.TalonFXFollower;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * A TalonFX that is logged. Construct {@link PhoenixTalonFX}, {@link BaseTalonFXSim}, or {@link
 * NoOppTalonFX} to use this class.
 */
public abstract class LoggedTalonFX {
  protected final String name;
  private final TalonFXInputsAutoLogged inputs = new TalonFXInputsAutoLogged();
  private final Alert[] connectionAlerts;
  private boolean tuning = false;

  private LoggedTunableNumber kPTunable = null;
  private LoggedTunableNumber kITunable = null;
  private LoggedTunableNumber kDTunable = null;
  private LoggedTunableNumber kGTunable = null;
  private LoggedTunableNumber kSTunable = null;
  private LoggedTunableNumber kVTunable = null;
  private LoggedTunableNumber kATunable = null;
  private Slot0Configs tunedConfigs = null;
  private LoggedTalonFX[] tuningFollowers = null;
  protected final int followers;

  public LoggedTalonFX(String name, int followers) {
    this.followers = followers;
    this.name = name;
    this.connectionAlerts = new Alert[followers + 1];
    this.connectionAlerts[0] =
        new Alert("TalonFX" + name + " is not connected", Alert.AlertType.kError);
    if (followers != 0) {
      for (int i = 1; i <= followers; i++) {
        connectionAlerts[i] =
            new Alert(
                "TalonFX " + name + " follower " + i + " is not connected", Alert.AlertType.kError);
      }
    }
    inputs.torqueCurrentAmps = new double[followers + 1];
    inputs.temperatureC = new double[followers + 1];
    inputs.connected = new boolean[followers + 1];
    inputs.supplyCurrentAmps = new double[followers + 1];
    inputs.appliedVolts = new double[followers + 1];
  }

  public void periodic() {
    this.updateInputs(inputs);
    Logger.processInputs("Motors/" + name, inputs);
    if (tuning) {
      LoggedTunableNumber.ifChanged(
          this,
          this::applyAllTuningChanges,
          kPTunable,
          kITunable,
          kDTunable,
          kGTunable,
          kSTunable,
          kVTunable,
          kATunable);
    }
    for (int i = 0; i < followers + 1; i++) {
      connectionAlerts[i].set(!inputs.connected[i]);
    }
  }

  private void applyAllTuningChanges(double[] values) {
    applyTuningChange(values);
    for (LoggedTalonFX tuningFollower : tuningFollowers) {
      tuningFollower.applyTuningChange(values);
    }
  }

  private void applyTuningChange(double[] values) {
    tunedConfigs.kP = values[0];
    tunedConfigs.kI = values[1];
    tunedConfigs.kD = values[2];
    tunedConfigs.kG = values[3];
    tunedConfigs.kS = values[4];
    tunedConfigs.kV = values[5];
    tunedConfigs.kA = values[6];
    quickApplySlot0Config(tunedConfigs);
  }

  public LoggedTalonFX withTunable(Slot0Configs defaultValues, LoggedTalonFX... followers) {
    if (!Constants.tuningMode) return this;
    kPTunable = new LoggedTunableNumber(name + "/kP", defaultValues.kP);
    kITunable = new LoggedTunableNumber(name + "/kI", defaultValues.kI);
    kDTunable = new LoggedTunableNumber(name + "/kD", defaultValues.kD);
    kGTunable = new LoggedTunableNumber(name + "/kG", defaultValues.kG);
    kSTunable = new LoggedTunableNumber(name + "/kS", defaultValues.kS);
    kVTunable = new LoggedTunableNumber(name + "/kV", defaultValues.kV);
    kATunable = new LoggedTunableNumber(name + "/kA", defaultValues.kA);

    tunedConfigs = defaultValues;
    tuningFollowers = followers;

    tuning = true;
    return this;
  }

  public abstract void setControl(ControlRequest controlRequest);

  protected abstract void updateInputs(TalonFXInputs inputs);

  public abstract LoggedTalonFX withConfig(TalonFXConfiguration config);

  /**
   * Apply a config for simulation. This is ignored by a real IO interface. This should be called
   * after a normal config is applied.
   *
   * @param config Function to generate a config. This will only be called in simulation. Takes in
   *     the current config, modifies it for simulation, and returns it. This is meant to make the
   *     simulation resemble reality as much as possible.
   * @return This for daisy-chaining
   */
  public abstract LoggedTalonFX withSimConfig(
      Function<TalonFXConfiguration, TalonFXConfiguration> config);

  public abstract void quickApplySlot0Config(Slot0Configs config);

  public Voltage getPrimaryAppliedVoltage() {
    return getAppliedVoltage(0);
  }

  public Voltage getAppliedVoltage(int follower) {
    return Volt.of(getAppliedVoltageVolts(follower));
  }

  public double getPrimaryAppliedVoltageVolts() {
    return getAppliedVoltageVolts(0);
  }

  public double getAppliedVoltageVolts(int follower) {
    return this.inputs.appliedVolts[follower];
  }

  public Temperature getPrimaryTemperature() {
    return getTempurature(0);
  }

  public Temperature getTempurature(int follower) {
    return Celsius.of(this.inputs.temperatureC[follower]);
  }

  public double getPrimaryTemperatureC() {
    return getTempuratureC(0);
  }

  public double getTempuratureC(int follower) {
    return this.inputs.temperatureC[follower];
  }

  public Current getPrimaryTorqueCurrent() {
    return getTorqueCurrent(0);
  }

  public Current getTorqueCurrent(int follower) {
    return Amp.of(getTorqueCurrentAmps(follower));
  }

  public double getPrimaryTorqueCurrentAmps() {
    return getTorqueCurrentAmps(0);
  }

  public double getTorqueCurrentAmps(int follower) {
    return this.inputs.torqueCurrentAmps[follower];
  }

  public Current getPrimarySupplyCurrent() {
    return getSupplyCurrent(0);
  }

  public Current getSupplyCurrent(int follower) {
    return Amp.of(getSupplyCurrentAmps(follower));
  }

  public double getPrimarySupplyCurrentAmps() {
    return getSupplyCurrentAmps(0);
  }

  public double getSupplyCurrentAmps(int follower) {
    return this.inputs.supplyCurrentAmps[follower];
  }

  public AngularVelocity getVelocity() {
    return this.inputs.velocity;
  }

  public Angle getPosition() {
    return this.inputs.position;
  }

  public abstract void setPosition(Angle position);
}

/*
LoggedTalon is an abstract clas
RealTalonFX implements LoggedTalon
HardwareTalonFX extends RealTalonFX
SimTalonFX extends RealTalonFX
 */
