// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import frc.robot.util.LoggedServo.LoggedServo;

/** Add your docs here. */
public class FunnelServoHub {
  private final ServoHub servoHub;
  private final int minPulse;
  private final int maxPulse;

  public FunnelServoHub(int CanID, int minPulse, int maxPulse) {
    this.minPulse = minPulse;
    this.maxPulse = maxPulse;
    ServoHubConfig config = new ServoHubConfig();
    config
        .channel0
        .pulseRange(minPulse, LoggedServo.calculateCenterPulse(minPulse, maxPulse), maxPulse)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

    // Persist parameters and reset any not explicitly set above to
    // their defaults.
    this.servoHub = new ServoHub(CanID);
    servoHub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
  }

  public ServoHubServo getServo(ChannelId channelId) {
    return new ServoHubServo(servoHub.getServoChannel(channelId), minPulse, maxPulse);
  }
}
