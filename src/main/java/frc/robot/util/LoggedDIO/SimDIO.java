package frc.robot.util.LoggedDIO;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class SimDIO extends LoggedDIO {

  private final BooleanSupplier simValue;

  public SimDIO(String name, BooleanSupplier simValue) {
    super(name);
    this.simValue = simValue;
  }

  @Override
  protected void updateInputs(DIOInputsAutoLogged inputs) {
    inputs.value = simValue.getAsBoolean();
  }

  public static SimDIO fromNT(String name) {
    LoggedNetworkBoolean networkValue =
        new LoggedNetworkBoolean("/Tuning/simInput/dio" + name, false);
    return new SimDIO(name, networkValue::get);
  }
}
