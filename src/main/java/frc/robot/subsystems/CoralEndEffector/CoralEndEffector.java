// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralEndEffector;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class CoralEndEffector extends SubsystemBase {
  private final LoggedTunableNumber intakeDutyCycle =
      new LoggedTunableNumber("Coral/Intake Duty Cycle", 0.30);
  private final LoggedTunableNumber outtakeDutyCycle =
      new LoggedTunableNumber("Coral/Outtake Duty Cycle", 0.25);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);
  private final StaticBrake brake = new StaticBrake();
  private final LoggedTalonFX talon;
  private final LoggedDIO beamBreak;
  private Debouncer beamBreakDebouncer = new Debouncer(0.25);
  @AutoLogOutput @Getter private boolean debouncedBeamBreakTripped = false;

  public CoralEndEffector(LoggedTalonFX talon, LoggedDIO beambreak) {
    var config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    this.talon = talon.withConfig(config);
    this.beamBreak = beambreak.withReversed(true);
  }

  public Command runIntake() {
    return runEnd(
        this::startIntake, // Start the motor
        this::stop // Stop the motor
        );
  }

  public Command runOuttake() {
    return runEnd(
        () -> talon.setControl(dutyCycleOut.withOutput(outtakeDutyCycle.get())), // Start the motor
        this::stop // Stop the motor
        );
  }
  public void startIntake() {
    talon.setControl(dutyCycleOut.withOutput(intakeDutyCycle.get()));
  }
  public void stop() {
    talon.setControl(brake);
  }

  public Trigger beamBreakTrigger() {
    return new Trigger(this::getBeamBreak);
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  }

  @Override
  public void periodic() {
    talon.periodic();
    beamBreak.periodic();
    debouncedBeamBreakTripped = beamBreakDebouncer.calculate(beamBreak.get());
    // This method will be called once per scheduler run
  }
}
