package frc.robot.util.LoggedTalon;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
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
  private boolean pidTuning = false;
  private boolean mmTuning = false;

  private LoggedTunableNumber[] tunableNumbers = null;
  private TalonFXConfiguration tunedConfigs = null;
  private LoggedTalonFX[] tuningFollowers = null;
  protected final int followers;

  private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
  private final MutAngle position = Radian.mutable(0);

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
    inputs.positionRot = 0;
    inputs.velocityRotPS = 0;
  }

  public void periodic() {
    this.updateInputs(inputs);
    Logger.processInputs("Motors/" + name, inputs);
    if (pidTuning) {
      LoggedTunableNumber.ifChanged(this, this::applyAllTuningChanges, tunableNumbers);
    }
    for (int i = 0; i < followers + 1; i++) {
      connectionAlerts[i].set(!inputs.connected[i]);
    }
  }

  private void applyAllTuningChanges(double[] values) {
    applyTuningChange(values, pidTuning, mmTuning);
    for (LoggedTalonFX tuningFollower : tuningFollowers) {
      tuningFollower.applyTuningChange(values, pidTuning, mmTuning);
    }
  }

  private void applyTuningChange(double[] values, boolean pid, boolean mm) {
    if (pid) {
      tunedConfigs.Slot0.kP = values[0];
      tunedConfigs.Slot0.kI = values[1];
      tunedConfigs.Slot0.kD = values[2];
      tunedConfigs.Slot0.kG = values[3];
      tunedConfigs.Slot0.kS = values[4];
      tunedConfigs.Slot0.kV = values[5];
      tunedConfigs.Slot0.kA = values[6];
    }
    if (mm) {
      tunedConfigs.MotionMagic.MotionMagicCruiseVelocity = values[7];
      tunedConfigs.MotionMagic.MotionMagicAcceleration = values[8];
      tunedConfigs.MotionMagic.MotionMagicJerk = values[9];
    }
    quickApplyConfig(tunedConfigs);
  }

  public LoggedTalonFX withPIDTunable(
      TalonFXConfiguration defaultValues, LoggedTalonFX... followers) {
    if (!Constants.tuningMode) return this;
    if (pidTuning) {
      DriverStation.reportWarning("Attempted to initiate PID tuning twice", true);
    }
    pidTuning = true;

    this.tunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(name + "/kP", defaultValues.Slot0.kP),
          new LoggedTunableNumber(name + "/kI", defaultValues.Slot0.kI),
          new LoggedTunableNumber(name + "/kD", defaultValues.Slot0.kD),
          new LoggedTunableNumber(name + "/kG", defaultValues.Slot0.kG),
          new LoggedTunableNumber(name + "/kS", defaultValues.Slot0.kS),
          new LoggedTunableNumber(name + "/kV", defaultValues.Slot0.kV),
          new LoggedTunableNumber(name + "/kA", defaultValues.Slot0.kA),
        };

    tunedConfigs = defaultValues;
    tuningFollowers = followers;

    return this;
  }

  public LoggedTalonFX withMMPIDTuning(
      TalonFXConfiguration defaultValues, LoggedTalonFX... followers) {
    withPIDTunable(defaultValues, followers);
    if (!Constants.tuningMode) return this;
    if (mmTuning) {
      DriverStation.reportWarning("Attempted to initiate Motion Magic tuning twice", true);
    }
    mmTuning = true;

    final LoggedTunableNumber[] mmTunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(
              name + "/MM Cruise Velocity", defaultValues.MotionMagic.MotionMagicCruiseVelocity),
          new LoggedTunableNumber(
              name + "/MM Acceleration", defaultValues.MotionMagic.MotionMagicAcceleration),
          new LoggedTunableNumber(name + "/MM Jerk", defaultValues.MotionMagic.MotionMagicJerk),
        };
    final LoggedTunableNumber[] copy =
        new LoggedTunableNumber[this.tunableNumbers.length + mmTunableNumbers.length];
    System.arraycopy(this.tunableNumbers, 0, copy, 0, this.tunableNumbers.length);
    System.arraycopy(
        mmTunableNumbers, 0, copy, this.tunableNumbers.length, mmTunableNumbers.length);
    this.tunableNumbers = copy;
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

  public abstract void quickApplyConfig(TalonFXConfiguration config);

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
    return velocity.mut_replace(this.inputs.velocityRotPS, RotationsPerSecond);
  }

  public Angle getPosition() {
    return position.mut_replace(this.inputs.positionRot, Rotation);
  }

  public abstract void setPosition(Angle position);
}

/*
LoggedTalon is an abstract clas
RealTalonFX implements LoggedTalon
HardwareTalonFX extends RealTalonFX
SimTalonFX extends RealTalonFX
 */
