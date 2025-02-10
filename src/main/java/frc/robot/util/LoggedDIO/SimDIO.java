package frc.robot.util.LoggedDIO;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;

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
    BooleanSubscriber subscriber =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("DigitalInput/" + name + "/simInput")
            .subscribe(false);
    return new SimDIO(name, subscriber);
  }
}
