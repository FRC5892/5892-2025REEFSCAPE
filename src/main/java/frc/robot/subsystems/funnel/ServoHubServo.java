// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.revrobotics.servohub.ServoChannel;
import frc.robot.util.LoggedServo.LoggedServo;

/** Add your docs here. */
public class ServoHubServo extends LoggedServo {
  private final ServoChannel channel;

  public ServoHubServo(ServoChannel channel, int minPulse, int maxPulse) {
    super(minPulse, maxPulse);
    this.channel = channel;
    channel.setEnabled(true);
    channel.setPowered(true);
  }

  @Override
  public void setPulse(int pulse) {
    channel.setPulseWidth(pulse);
  }

  @Override
  public void setEnabled(boolean enabled) {
    channel.setEnabled(enabled);
  }

  @Override
  public void setPowered(boolean powered) {
    channel.setPowered(powered);
  }
}
