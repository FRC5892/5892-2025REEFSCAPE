// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedServo.LoggedServo;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final LoggedTunableNumber upPosition = new LoggedTunableNumber("Funnel/upPosition", 800);
  private final LoggedTunableNumber downPosition =
      new LoggedTunableNumber("Funnel/downPosition", 2200);
  private final LoggedServo servo;

  public Funnel(LoggedServo servo) {
    this.servo = servo;
  }

  public Command foldUp() {
    return runOnce(
        () -> {
          logAndSetPercent(upPosition.get());
        });
  }

  public Command foldDown() {
    return runOnce(
        () -> {
          logAndSetPercent(downPosition.get());
        });
  }

  public void logAndSetPercent(double percent) {
    Logger.recordOutput("Funnel/setpoint", percent);
    servo.setEnabled(true);
    servo.setPowered(true);
    servo.setPulse((int) percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  //
  // 2200
  // 800
}
