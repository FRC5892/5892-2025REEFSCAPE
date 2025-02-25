// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedServo;

/** Add your docs here. */
public class NoOppServo extends LoggedServo {

  public NoOppServo(int minPulse, int maxPulse) {
    super(minPulse, maxPulse);
  }

  @Override
  public void setPulse(int pulse) {}

  @Override
  public void setEnabled(boolean enabled) {}

  @Override
  public void setPowered(boolean powered) {}
}
