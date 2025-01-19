package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.units.measure.Voltage;

/**
 * A TalonFX that is logged. Create A {@link PhoenixTalonFX} or {@link TalonFXSim} to use this class.
 */
public abstract class LoggedTalonFX {
  protected final String name;
  private final TalonFXInputs inputs = new TalonFXInputs();

  public LoggedTalonFX(String name) {
    this.name = name;
  }

  public void periodic() {
    this.updateInputs(inputs);
  }

  public abstract void runVoltage(Voltage voltage);

  protected abstract void updateInputs(TalonFXInputs inputs);
}

/*
LoggedTalon is an abstract clas
RealTalonFX implements LoggedTalon
HardwareTalonFX extends RealTalonFX
SimTalonFX extends RealTalonFX
 */
