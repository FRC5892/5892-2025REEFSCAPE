// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class TunablePIDController extends PIDController {
    private final LoggedTunableNumber[] tunableNumbers;
    public TunablePIDController(String key, PIDConstants constants) {
        super(constants.kP, constants.kI,constants.kD);
        super.setIZone(constants.iZone);
        tunableNumbers = new LoggedTunableNumber[]{
            new LoggedTunableNumber(key+"/kP", constants.kP),
            new LoggedTunableNumber(key+"/kP", constants.kP),
            new LoggedTunableNumber(key+"/kP", constants.kP),
        }

    }
}
