// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralEndEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;

public class CoralEndEffector extends SubsystemBase {
  private final LoggedTunableNumber dutyCycle = new LoggedTunableNumber("Coral/Duty Cycle", 0.75);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);
  private final NeutralOut coastOut = new NeutralOut();
  private final LoggedTalonFX talon;
  private final LoggedDIO beamBreak;

  public CoralEndEffector(LoggedTalonFX talon, LoggedDIO beambreak) {
    var config = new TalonFXConfiguration();
    this.talon = talon.withConfig(config).withTunable(config.Slot0);
    this.beamBreak = beambreak;
  }

  public Command runAtDutyCycle(double dutyCycle) {
    return runEnd(
        () -> talon.setControl(dutyCycleOut.withOutput(dutyCycle)), // Start the motor
        () -> talon.setControl(coastOut) // Stop the motor
        );
  }

  public Command runIntake() {
    return runAtDutyCycle(dutyCycle.get());
  }

  public Trigger beamBreakTrigger() {
    return new Trigger(beamBreak::get);
  }

  @Override
  public void periodic() {
    talon.periodic();
    // This method will be called once per scheduler run
  }
}
