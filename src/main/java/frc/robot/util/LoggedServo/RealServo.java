// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedServo;

import edu.wpi.first.wpilibj.Servo;

public class RealServo extends LoggedServo {
    private final Servo servo;
    public RealServo(int id) {
        this.servo = new Servo(id);
    }
    @Override
    public void setPosition(double position) {
        servo.setPosition(position);
    }

    @Override
    public void setAngle(double degrees) {
        servo.setAngle(degrees);
    }
}
