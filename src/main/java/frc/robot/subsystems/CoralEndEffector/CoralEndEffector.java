// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralEndEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

public class CoralEndEffector extends SubsystemBase {
  private static final double DEFAULT_DUTY_CYCLE = 0.5;
  private final DutyCycleOut climbDutyCycleOut = new DutyCycleOut(DEFAULT_DUTY_CYCLE);
  private final DutyCycleOut stopDutyCycleOut = new DutyCycleOut(0);
  private final LoggedTalonFX talon;
  // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/control-requests.html#modifying-a-control-request
  public CoralEndEffector(LoggedTalonFX talon) {
    var config = new TalonFXConfiguration();
    this.talon =
        talon
            .withPosition()
            .withVelocity()
            .withAppliedVoltage()
            .withConfig(config)
            .withTunable(config.Slot0);
  }

  public Command runAtDutyCycle() {
    return runEnd(
        () -> talon.setControl(climbDutyCycleOut), // Start the motor
        () -> talon.setControl(stopDutyCycleOut)); // Stop the motor
  }

  @Override
  public void periodic() {
    talon.periodic();
    talon.setControl(new DutyCycleOut(DEFAULT_DUTY_CYCLE));

    // This method will be called once per scheduler run
  }
}
