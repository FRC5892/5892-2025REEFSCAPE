// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class TunablePIDController extends PIDController {
  private final LoggedTunableNumber[] tunableNumbers;

  public TunablePIDController(String key, PIDConstants constants, double period) {
    super(constants.kP, constants.kI, constants.kD, period);
    super.setIntegratorRange(-constants.iZone, constants.iZone);
    tunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(key + "/kP", constants.kP),
          new LoggedTunableNumber(key + "/kI", constants.kI),
          new LoggedTunableNumber(key + "/kD", constants.kD),
        };
  }

  public TunablePIDController(String key, PIDConstants constants) {
    this(key, constants, 0.02);
  }

  public void updateConstants(Object id, PIDController... tuningFollowers) {
    LoggedTunableNumber.ifChanged(
        id,
        constants -> {
          super.setPID(constants[0], constants[1], constants[2]);
          for (PIDController follower : tuningFollowers) {
            follower.setPID(constants[0], constants[1], constants[2]);
          }
        },
        tunableNumbers);
  }
}
