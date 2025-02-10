// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedDIO;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class HardwareDIO extends LoggedDIO {
  private final DigitalInput hardware;

  public HardwareDIO(String name, int id) {
    super(name);
    hardware = new DigitalInput(id);
  }

  @Override
  protected void updateInputs(DIOInputsAutoLogged inputs) {
    inputs.value = hardware.get();
  }
}
