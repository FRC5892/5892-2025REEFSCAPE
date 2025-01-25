package frc.robot.util.LoggedTalon.Follower;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class PhoenixTalonFXFollower {
    private final TalonFX talonFX;
    boolean inverted;

    public PhoenixTalonFXFollower(int canID, CANBus canBus, boolean inverted) {
        talonFX = new TalonFX(canID, canBus);
        this.inverted = inverted;
    }

    public void follow(TalonFX leader) {
        talonFX.setControl(new Follower(leader.getDeviceID(),inverted));
    }

}
