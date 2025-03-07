// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedServo.LoggedServo;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final LoggedServo servo;

  public Funnel(LoggedServo servo) {
    this.servo = servo;
  }

  public Command move(FunnelPosition position) {
    return runOnce(
        () -> {
          logAndSetPercent(position.getPosition().getAsDouble());
        });
  }

  @Getter
  @RequiredArgsConstructor
  public enum FunnelPosition {
    UP(new LoggedTunableNumber("Funnel/upPosition", 650)),
    STARTING(new LoggedTunableNumber("Funnel/startingPosition", 2000)),
    DOWN(new LoggedTunableNumber("Funnel/downPosition", 2500));
    private final DoubleSupplier position;
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
