package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  private final LoggedTalonFX talon;

  private final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(0, 3);
  private final LoggedMechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Elevator Root", 0, 0);
  private final LoggedMechanismLigament2d mechanism2dLigament =
      mechanism2dRoot.append(new LoggedMechanismLigament2d("Elevator Ligament", 0, 90));

  private final double ELEVATOR_SPOOL_DIAMETER = 0.005 * 2;
  private final double ELEVATOR_GEAR_RATIO = 3;

  private final VoltageOut voltageOut = new VoltageOut(0);

  private double heightMeters = -1;

  public Elevator(LoggedTalonFX talon) {
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

  @Override
  public void periodic() {
    talon.periodic();
    heightMeters = rotationsToMeters(talon.getPosition().in(Rotations));
    mechanism2dLigament.setLength(heightMeters);
    Logger.recordOutput("Elevator/mechanism", mechanism2d);
    Logger.recordOutput("Elevator/height", heightMeters);
  }

  public Command moveDutyCycle(DoubleSupplier dutyCycle) {
    return runEnd(
        () -> {
          voltageOut.withOutput(dutyCycle.getAsDouble() * 12);
          Logger.recordOutput("Elevator/VoltageOut", voltageOut.Output);
          talon.setControl(voltageOut);
        },
        () -> talon.setControl(voltageOut.withOutput(0)));
  }

  private double metersToRotations(double heightMeters) {
    return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) / ELEVATOR_GEAR_RATIO;
  }

  private double rotationsToMeters(double rotations) {
    return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) * ELEVATOR_GEAR_RATIO;
  }
  @AutoLogOutput(key = "Elevator/height")
  public double getHeightMeters() {
    return heightMeters;
  }
}
