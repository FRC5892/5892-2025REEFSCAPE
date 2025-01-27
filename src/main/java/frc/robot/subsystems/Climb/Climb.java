// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

public class Climb extends SubsystemBase {
  private final LoggedTalonFX talon;
  // change val if needed
  public static final double DEFAULT_DUTY_CYCLE = 0.75;
  public final DutyCycleOut ClimbDutyCycle = new DutyCycleOut(DEFAULT_DUTY_CYCLE);
  public final DutyCycleOut StopDutyCycleOut = new DutyCycleOut(0);

  public Climb(LoggedTalonFX talon) {
    var config = new TalonFXConfiguration();
    this.talon =
        talon
            .withPosition()
            .withVelocity()
            .withAppliedVoltage()
            .withConfig(config)
            .withSimConfig(
                c ->
                    c.withCurrentLimits(
                        new CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(false)
                            .withSupplyCurrentLimitEnable(false)))
            .withTunable(config.Slot0);
  }

  public Command runAtDutyCycle() {
    return runEnd(() -> talon.setControl(ClimbDutyCycle), () -> talon.setControl(StopDutyCycleOut));
  }

  @Override
  public void periodic() {
    talon.periodic();
    talon.setControl(new DutyCycleOut(DEFAULT_DUTY_CYCLE));
  }
}
