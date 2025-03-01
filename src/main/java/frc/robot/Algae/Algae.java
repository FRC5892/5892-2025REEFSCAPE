package frc.robot.Algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;

public class Algae extends SubsystemBase {
    private final LoggedTalonFX rollerMotor;
    private final LoggedTalonFX pivotMotor;

    private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0).withEnableFOC(true);

    private final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Algae/Intake Speed", 0.5);
    private final LoggedTunableNumber outtakeSpeed = new LoggedTunableNumber("Algae/Outtake Speed", -0.5);

    private final LoggedTunableNumber stowPosition = new LoggedTunableNumber("Algae/Stow Position", 0);
    private final LoggedTunableNumber reefIntakePosition = new LoggedTunableNumber("Algae/Reef Intake Position", 2);

    public Algae(LoggedTalonFX rollerMotor, LoggedTalonFX pivotMotor) {
        this.rollerMotor = rollerMotor.withConfig(new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(40)
        ));
        var pivotConfig = new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withStatorCurrentLimit(40)
        ).withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0)).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(5).withMotionMagicCruiseVelocity(10));
        this.pivotMotor = pivotMotor.withConfig(pivotConfig).withMMPIDTuning(pivotConfig);
    }

    @Override
    public void periodic() {
        rollerMotor.periodic();
        pivotMotor.periodic();
        // This method will be called once per scheduler run
    }
}
