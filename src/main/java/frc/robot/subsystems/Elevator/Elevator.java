package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  private final LoggedTalonFX talon;
  @AutoLogOutput private final LoggedMechanism2d mechanism = new LoggedMechanism2d(0, 3);
  private final LoggedMechanismRoot2d mechanism2dRoot = mechanism.getRoot("Elevator Root", 0, 0);
  private final LoggedMechanismLigament2d mechanism2dLigament =
      mechanism2dRoot.append(new LoggedMechanismLigament2d("Elevator Ligament", 0, 90));

  private final VoltageOut voltageOut = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicControl =
      new MotionMagicVoltage(Degrees.zero()).withEnableFOC(true);
  @AutoLogOutput @Getter private Distance height = Meters.zero();
  @AutoLogOutput private ElevatorPosition setPoint = ElevatorPosition.INTAKE;
  @Getter @AutoLogOutput private boolean atSetpoint = false;

  public Elevator(LoggedTalonFX talon) {
    var config =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(20)
                    .withMotionMagicAcceleration(100)
                    .withMotionMagicJerk(100))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(10).withStatorCurrentLimit(20))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO));
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

  @Override
  public void periodic() {
    talon.periodic();
    height = Elevator.angleToDistance(talon.getPosition());
    atSetpoint =
        MathUtil.isNear(
                setPoint.height.get().baseUnitMagnitude(),
                height.baseUnitMagnitude(),
                ElevatorConstants.DISTANCE_TOLERANCE_METERS)
            && talon.getVelocity().in(RPM) < ElevatorConstants.VELOCITY_TOLERANCE_RPM;
    mechanism2dLigament.setLength(height.in(Meters));
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

  /**
   * Moves the elevator to a position.
   *
   * @param position the position.
   * @return the command
   */
  public Command goToPosition(ElevatorPosition position) {
    return runOnce(
        () -> {
          talon.setControl(motionMagicControl.withPosition(distanceToAngle(position.height.get())));
          Logger.recordOutput("Elevator/MotorPositionSetpoint", motionMagicControl.Position);
          setPoint = position;
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
