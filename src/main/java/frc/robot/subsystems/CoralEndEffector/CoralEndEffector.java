// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralEndEffector;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;

public class CoralEndEffector extends SubsystemBase {
  private final LoggedTunableNumber intakeDutyCycle =
      new LoggedTunableNumber("Coral/Intake Duty Cycle", 0.1);
  private final LoggedTunableNumber outtakeDutyCycle =
      new LoggedTunableNumber("Coral/Outtake Duty Cycle", 0.25);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);
  private final StaticBrake brake = new StaticBrake();
  private final LoggedTalonFX talon;
  private final LoggedDIO beamBreak;

  public CoralEndEffector(LoggedTalonFX talon, LoggedDIO beambreak) {
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    this.talon = talon.withConfig(config);
    this.beamBreak = beambreak;
  }

  public Command runIntake() {
    return runEnd(
        () -> talon.setControl(dutyCycleOut.withOutput(intakeDutyCycle.get())), // Start the motor
        () -> talon.setControl(brake) // Stop the motor
        );
  }

  public Command runOuttake() {
    return runEnd(
        () -> talon.setControl(dutyCycleOut.withOutput(outtakeDutyCycle.get())), // Start the motor
        () -> talon.setControl(brake) // Stop the motor
        );
  }

  public Trigger beamBreakTrigger() {
    return new Trigger(this::getBeamBreak);
  }

  public boolean getBeamBreak() {
    return !beamBreak.get();
  }

  @Override
  public void periodic() {
    talon.periodic();
    beamBreak.periodic();
    // This method will be called once per scheduler run
  }
}
