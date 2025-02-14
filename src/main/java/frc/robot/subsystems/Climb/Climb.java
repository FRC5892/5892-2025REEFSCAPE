// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;

public class Climb extends SubsystemBase {
  private final LoggedTalonFX talon;
  private final LoggedTunableNumber climbSpeed = new LoggedTunableNumber("Climb/dutyCycle", 0.25);
  public final DutyCycleOut climbDutyCycle = new DutyCycleOut(0);
  public final StaticBrake brake = new StaticBrake();

  public Climb(LoggedTalonFX talon) {
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40));
    this.talon =
        talon
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
    return runEnd(
        () -> talon.setControl(climbDutyCycle.withOutput(climbSpeed.get())),
        () -> talon.setControl(brake));
  }

  @Override
  public void periodic() {
    talon.periodic();
  }
}
