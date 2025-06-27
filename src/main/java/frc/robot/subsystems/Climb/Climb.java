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
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final LoggedTalonFX talon;
  private final LoggedTunableNumber climbExtendSpeed =
      new LoggedTunableNumber("Climb/ExtendDutyCycle", 1);
  private final LoggedTunableNumber climbRetractSpeed =
      new LoggedTunableNumber("Climb/RetractDutyCycle", -1);

  private final DutyCycleOut climbDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final StaticBrake brake = new StaticBrake();

  private final LoggedDIO forwardLimit;
  private final LoggedDIO reverseLimit;

  private final TalonFXConfiguration config;

  public Climb(LoggedTalonFX talon, LoggedDIO forwardLimit, LoggedDIO reverseLimit) {
    this.forwardLimit = forwardLimit.withReversed(true);
    this.reverseLimit = reverseLimit.withReversed(true);
    this.config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(150).withSupplyCurrentLimit(60));
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
            () -> talon.setControl(brake))
        .until(forwardLimit::get);
  }

  public Command climbExtend() {
    return runEnd(
            () ->
                talon.setControl(
                    climbDutyCycle
                        .withOutput(climbExtendSpeed.getAsDouble())
                        .withLimitForwardMotion(forwardLimit.get())
                        .withLimitReverseMotion(reverseLimit.get())),
            () -> talon.setControl(brake))
        .until(forwardLimit::get);
  }

  public Command climbRetract() {
    return runEnd(
            () ->
                talon.setControl(
                    climbDutyCycle
                        .withOutput(climbRetractSpeed.getAsDouble())
                        .withLimitForwardMotion(forwardLimit.get())
                        .withLimitReverseMotion(reverseLimit.get())),
            () -> talon.setControl(brake))
        .until(reverseLimit::get);
  }

  public Command coastCommand() {
    return runOnce(
            () -> {
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              new Thread(() -> talon.quickApplyConfig(config)).start();
            })
        .ignoringDisable(true);
  }

  public Command brakeCommand() {
    return runOnce(
            () -> {
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              new Thread(() -> talon.quickApplyConfig(config)).start();
            })
        .ignoringDisable(true);
  }

  @Override
  public void periodic() {
    talon.periodic();
    forwardLimit.periodic();
    reverseLimit.periodic();
    Logger.recordOutput("Climb/forwardLimit", forwardLimit.get());
    Logger.recordOutput("Climb/reverseLimit", reverseLimit.get());
  }
}
