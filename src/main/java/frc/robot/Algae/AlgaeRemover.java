package frc.robot.Algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedServo.LoggedServo;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlgaeRemover extends SubsystemBase {
  private final LoggedServo algaeServo;
  private final LoggedTalonFX algaeMotor;

  private final LoggedTunableNumber speed = new LoggedTunableNumber("Algae/Speed", 0.5);
  private final LoggedTunableNumber extendPosition =
      new LoggedTunableNumber("Algae/ExtendPosition", 800);
  private final LoggedTunableNumber retractPosition =
      new LoggedTunableNumber("Algae/RetractPosition", 0);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public AlgaeRemover(LoggedServo algaeServo, LoggedTalonFX algaeMotor) {
    this.algaeServo = algaeServo;
    this.algaeMotor = algaeMotor;

    var motorConfig = new TalonFXConfiguration();
    this.algaeMotor.withConfig(
        motorConfig.withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(30)));
  }

  public Command extendServo() {
    return setServo(extendPosition);
  }

  public Command retractServo() {
    return setServo(retractPosition);
  }

  public Command runMotor() {
    return runEnd(
        () -> {
          algaeMotor.setControl(dutyCycleOut.withOutput(speed.getAsDouble()));
        },
        () -> {
          algaeMotor.setControl(dutyCycleOut.withOutput(0));
        });
  }

  public Command setServo(DoubleSupplier pulse) {
    return runOnce(
        () -> {
          algaeServo.setEnabled(true);
          algaeServo.setPowered(true);
          logAndSetServo((int) pulse.getAsDouble());
        });
  }

  public void logAndSetServo(int pulse) {
    Logger.recordOutput("Algae/ServoSetpoint", pulse);
    algaeServo.setPulse(pulse);
  }

  @Override
  public void periodic() {
    algaeMotor.periodic();
  }
}
