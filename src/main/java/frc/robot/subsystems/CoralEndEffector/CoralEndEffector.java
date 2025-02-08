// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralEndEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

public class CoralEndEffector extends SubsystemBase {
  private static final double DEFAULT_DUTY_CYCLE = 0.5;
  private final DutyCycleOut dutyCycleOut =
      new DutyCycleOut(DEFAULT_DUTY_CYCLE).withEnableFOC(true);
  private final CoastOut coastOut = new CoastOut();
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

  public Command intakeCommand() {
    return new WaitUntilCommand(() -> beamBreak.get())
        .andThen(runAtDutyCycle(DEFAULT_DUTY_CYCLE).until(() -> !beamBreak.get()));
  }

  @Override
  public void periodic() {
    talon.periodic();
    talon.setControl(new DutyCycleOut(DEFAULT_DUTY_CYCLE));

    // This method will be called once per scheduler run
  }
}
