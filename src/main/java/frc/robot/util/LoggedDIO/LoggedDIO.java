// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedDIO;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
@RequiredArgsConstructor
public abstract class LoggedDIO {
  private final DIOInputsAutoLogged inputs = new DIOInputsAutoLogged();
  private final String name;

  public void periodic() {
    updateInputs(inputs);
    Logger.processInputs("DigitalInput/" + name, inputs);
  }

  protected abstract void updateInputs(DIOInputsAutoLogged inputs);

  @AutoLog
  protected static class DIOInputs {
    boolean value;
  }

  public boolean get() {
    return inputs.value;
  }
}
