package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * A TalonFX that is logged. Construct {@link PhoenixTalonFX}, {@link TalonFXSim}, or {@link
 * NoOppTalonFX} to use this class.
 */
public abstract class LoggedTalonFX {

  protected final String name;
  private final TalonFXInputsAutoLogged inputs = new TalonFXInputsAutoLogged();
  private final Alert connectionAlert;
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

  public LoggedTalonFX(String name) {
    this.name = name;
    this.connectionAlert =
        new Alert("TalonFX" + name + " is not connected", Alert.AlertType.kError);
  }

  public void periodic() {
    this.updateInputs(inputs);
    Logger.processInputs("Motors/" + name, inputs);
    if (tuning) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          this::applyAllTuningChanges,
          kPTunable,
          kITunable,
          kDTunable,
          kGTunable,
          kSTunable,
          kVTunable,
          kATunable);
    }
    connectionAlert.set(inputs.connected);
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

  public abstract LoggedTalonFX withAppliedVoltage();

  public abstract LoggedTalonFX withTorqueCurrent();

  public abstract LoggedTalonFX withStatorCurrent();

  public abstract LoggedTalonFX withVelocity();

  public abstract LoggedTalonFX withPosition();

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

  public Voltage getAppliedVoltage() {
    return this.inputs.appliedVoltage;
  }

  public Current getTorqueCurrent() {
    return this.inputs.torqueCurrent;
  }

  public Current getStatorCurrent() {
    return this.inputs.statorCurrent;
  }

  public AngularVelocity getVelocity() {
    return this.inputs.velocity;
  }

  public Angle getPosition() {
    return this.inputs.position;
  }
}

/*
LoggedTalon is an abstract clas
RealTalonFX implements LoggedTalon
HardwareTalonFX extends RealTalonFX
SimTalonFX extends RealTalonFX
 */
