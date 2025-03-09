package frc.robot.util.LoggedTalon;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXInputs {
  public boolean[] connected;
  public double[] appliedVolts;
  public double[] torqueCurrentAmps;
  public double[] supplyCurrentAmps;
  public double[] temperatureC;
  public double velocityRotPS;
  public double positionRot;
}
