// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedServo.LoggedServo;

public class Funnel extends SubsystemBase {
  private final double FOLD_UP_POSITION = 1.0;
  private final LoggedServo servo;

  public Funnel(LoggedServo servo) {
    this.servo = servo;
  }

  public Command foldUp() {
    return runOnce(
        () -> {
          servo.setPosition(FOLD_UP_POSITION);
        });
  }

  public Command foldDown() {
    return runOnce(
        () -> {
          servo.setPosition(0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
