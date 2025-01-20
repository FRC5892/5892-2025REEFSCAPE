package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * A TalonFX that is logged. Construct {@link PhoenixTalonFX} or {@link TalonFXSim} to use this
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

  public abstract LoggedTalonFX withAppliedVoltage();

  public abstract LoggedTalonFX withTorqueCurrent();

  public abstract LoggedTalonFX withStatorCurrent();

  public abstract LoggedTalonFX withVelocity();

  public abstract LoggedTalonFX withPosition();

  public abstract void applyConfig(TalonFXConfiguration config);

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
