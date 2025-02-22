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
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private final LoggedTalonFX talon;
  private final LoggedTunableNumber climbExtendSpeed =
      new LoggedTunableNumber("Climb/ExtendDutyCycle", 0.25);
  private final LoggedTunableNumber climbRetractSpeed =
      new LoggedTunableNumber("Climb/RetractDutyCycle", -0.25);

  private final DutyCycleOut climbDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final StaticBrake brake = new StaticBrake();

  private final LoggedDIO forwardLimit;
  private final LoggedDIO reverseLimit;

  public Climb(LoggedTalonFX talon, LoggedDIO forwardLimit, LoggedDIO reverseLimit) {
    this.forwardLimit = forwardLimit;
    this.reverseLimit = reverseLimit;
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
                            .withSupplyCurrentLimitEnable(false)));
  }

  public Command runAtDutyCycle(DoubleSupplier dutyCycleSupplier) {
    return runEnd(
        () ->
            talon.setControl(
                climbDutyCycle
                    .withOutput(dutyCycleSupplier.getAsDouble())
                    .withLimitForwardMotion(forwardLimit.get())
                    .withLimitReverseMotion(reverseLimit.get())),
        () -> talon.setControl(brake));
  }

  public Command climbExtend() {
    return runAtDutyCycle(climbExtendSpeed).until(forwardLimit::get);
  }

  public Command climbRetract() {
    return runAtDutyCycle(climbRetractSpeed).until(reverseLimit::get);
  }

  @Override
  public void periodic() {
    talon.periodic();
  }
}
