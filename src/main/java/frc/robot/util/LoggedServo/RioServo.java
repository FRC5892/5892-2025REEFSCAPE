// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedServo;

import edu.wpi.first.wpilibj.Servo;

public class RioServo extends LoggedServo {
  private final Servo servo;

  public RioServo(int id, int maxPulse, int minPulse) {
    super(minPulse, maxPulse);
    this.servo = new Servo(id);
  }

  @Override
  public void setPulse(int pulse_us) {
    servo.setPulseTimeMicroseconds(pulse_us);
  }
}
