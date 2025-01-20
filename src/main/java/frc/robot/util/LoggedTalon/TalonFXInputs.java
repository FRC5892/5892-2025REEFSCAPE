package frc.robot.util.LoggedTalon;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXInputs {
  public boolean connected;
  public Voltage appliedVoltage;
  public Current torqueCurrent;
  public Current statorCurrent;
  public AngularVelocity velocity;
  public Angle position;
}
