package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Elevator extends SubsystemBase {
  private final LoggedTalonFX talon;
  private final LoggedTunableMeasure<MutAngularVelocity> homedThreshold =
      new LoggedTunableMeasure<>("Elevator/homing/Threshold", RotationsPerSecond.mutable(0.25));
  private final LoggedTunableNumber homingVoltage =
      new LoggedTunableNumber("Elevator/homing/Speed V", -0.5);

  private final LoggedNetworkBoolean eStop = new LoggedNetworkBoolean("Elevator/E Stop", false);
  private final LoggedTunableMeasure<MutDistance> tolerance =
      new LoggedTunableMeasure<>("Elevator/Tolerance", Meters.mutable(0.02));
  private final LoggedTunableMeasure<MutAngularVelocity> toleranceVelocity =
      new LoggedTunableMeasure<>("Elevator/ToleranceVelocity", RPM.mutable(1));
  @AutoLogOutput private final LoggedMechanism2d mechanism = new LoggedMechanism2d(0.25, 3);
  private final LoggedMechanismRoot2d mechanism2dRoot = mechanism.getRoot("Elevator Root", 0, 0);
  private final LoggedMechanismLigament2d mechanism2dLigament =
      mechanism2dRoot.append(new LoggedMechanismLigament2d("Elevator Ligament", 0, 90));

  private final VoltageOut voltageOut = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicControl =
      new MotionMagicVoltage(Degrees.zero()).withEnableFOC(true);

  @AutoLogOutput @Getter private final MutDistance height = Meters.mutable(0);
  @AutoLogOutput private ElevatorPosition setPoint = ElevatorPosition.INTAKE;
  @Getter @AutoLogOutput private boolean atSetpoint = false;
  @AutoLogOutput private boolean homed = false;
  private final Alert homedAlert = new Alert("Elevator not homed", Alert.AlertType.kWarning);
  private Debouncer homingDebouncer;

  public Elevator(LoggedTalonFX talon) {
    var config =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(20)
                    .withMotionMagicAcceleration(100)
                    .withMotionMagicJerk(100))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withStatorCurrentLimit(40))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKG(0.26).withKV(1.15).withKP(0.7).withKA(0.001));
    this.talon =
        talon
            .withConfig(config)
            .withSimConfig(
                c ->
                    c.withCurrentLimits(
                            new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(false)
                                .withSupplyCurrentLimitEnable(false))
                        .withSlot0(
                            new Slot0Configs().withKG(0.15).withKV(1.29).withKA(0.03).withKP(2.5)))
            .withMMPIDTuning(config, talon);
  }

  @Override
  public void periodic() {
    talon.periodic();
    height.mut_replace(Elevator.angleToDistance(talon.getPosition()));
    atSetpoint =
        MathUtil.isNear(
                setPoint.height.get().baseUnitMagnitude(),
                height.baseUnitMagnitude(),
                tolerance.get().baseUnitMagnitude())
            && MathUtil.isNear(
                0,
                talon.getVelocity().baseUnitMagnitude(),
                toleranceVelocity.get().baseUnitMagnitude());
    mechanism2dLigament.setLength(height.in(Meters));
    homedAlert.set(!homed);
  }

  public Command moveDutyCycle(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              if (!eStop.get()) {
                voltageOut.withOutput(dutyCycle.getAsDouble() * 12);
              } else {
                voltageOut.withOutput(0);
              }
              Logger.recordOutput("Elevator/VoltageOut", voltageOut.Output);

              talon.setControl(voltageOut);
            },
            () -> talon.setControl(voltageOut.withOutput(0)))
        .unless(eStop::get);
  }

  /**
   * Moves the elevator to a position.
   *
   * @param position the position.
   * @return the command
   */
  public Command goToPosition(ElevatorPosition position) {
    return runOnce(
            () -> {
              if (!eStop.get()) {
                talon.setControl(
                    motionMagicControl.withPosition(distanceToAngle(position.height.get())));
              } else {
                talon.setControl(voltageOut.withOutput(0));
              }
              Logger.recordOutput(
                  "Elevator/MotorPositionSetpoint", Rotations.of(motionMagicControl.Position));
              setPoint = position;
            })
        .unless(eStop::get)
        .andThen(Commands.waitUntil(this::isAtSetpoint));
  }

  public Command homeCommand() {
    return startRun(
            () -> {
              homingDebouncer = new Debouncer(0.25);
            },
            () -> {
              if (!eStop.get()) {
                talon.setControl(voltageOut.withOutput(homingVoltage.get()));
              } else {
                talon.setControl(voltageOut.withOutput(0));
              }
              homed =
                  homingDebouncer.calculate(
                      Math.abs(talon.getVelocity().baseUnitMagnitude())
                          <= homedThreshold.get().baseUnitMagnitude());
            })
        .unless(eStop::get)
        .until(() -> homed)
        .finallyDo(
            () -> {
              // Success or interrupted, just clean up
              talon.setControl(voltageOut.withOutput(0));
              // Let the GC take it away
              homingDebouncer = null;
            })
        .andThen(Commands.waitSeconds(1))
        .andThen(
            () -> {
              // Success!
              talon.setPosition(Rotations.of(0));
              homed = true;
            });
  }

  protected static Angle distanceToAngle(Distance height) {
    return Rotations.of(
        height.baseUnitMagnitude() / ElevatorConstants.DISTANCE_PER_ROTATION.baseUnitMagnitude());
  }

  protected static Distance angleToDistance(Angle angle) {
    return ElevatorConstants.DISTANCE_PER_ROTATION.times(angle.in(Rotations));
  }

  public boolean atPosition(ElevatorPosition position) {
    return position == setPoint && atSetpoint;
  }
}
