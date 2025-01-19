package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;

public class PhoenixTalonFX extends LoggedTalonFX {
  protected final TalonFX talonFX;
  private final Alert connectionAlert =
      new Alert("TalonFX" + name + " is not connected", Alert.AlertType.kError);

  public PhoenixTalonFX(int CAN_ID, CANBus canBus, String name) {
    super(name);
    talonFX = new TalonFX(CAN_ID, canBus);
  }

  @Override
  public void runVoltage(Voltage voltage) {}

  @Override
  protected void updateInputs(TalonFXInputs inputs) {}
}
