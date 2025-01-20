package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * A TalonFX that is logged. Create A {@link PhoenixTalonFX} or {@link TalonFXSim} to use this
 * class.
 */
public abstract class LoggedTalonFX {

  protected final String name;
  private final TalonFXInputsAutoLogged inputs = new TalonFXInputsAutoLogged();
  private final Alert connectionAlert;

  public LoggedTalonFX(String name) {
    this.name = name;
    this.connectionAlert =
        new Alert("TalonFX" + name + " is not connected", Alert.AlertType.kError);
  }

  public void periodic() {

    this.updateInputs(inputs);
    Logger.processInputs("Motors/" + name, inputs);
    connectionAlert.set(inputs.connected);
  }

  public abstract void setControl(ControlRequest controlRequest);

  protected abstract void updateInputs(TalonFXInputs inputs);

  protected abstract void withAppliedVoltage();

  protected abstract void withTorqueCurrent();

  protected abstract void withStatorCurrent();

  protected abstract void withVelocity();

  protected abstract void withPosition();

  public enum ImplementedSignals {
    AppliedVoltage(),
    TorqueCurrent,
    StatorCurrent,
    Velocity,
    Position,
  }

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
