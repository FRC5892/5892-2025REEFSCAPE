// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class TunableProfiledPIDController extends ProfiledPIDController
    implements Tunable<ProfiledPIDController> {
  private final LoggedTunableNumber[] tunableNumbers;

  public TunableProfiledPIDController(
      String key, PIDConstants constants, TrapezoidProfile.Constraints constraints, double period) {
    super(constants.kP, constants.kI, constants.kD, constraints, period);
    super.setIntegratorRange(-constants.iZone, constants.iZone);
    tunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(key + "/kP", constants.kP),
          new LoggedTunableNumber(key + "/kI", constants.kI),
          new LoggedTunableNumber(key + "/kD", constants.kD),
        };
  }

  public TunableProfiledPIDController(
      String key, PIDConstants constants, TrapezoidProfile.Constraints constraints) {
    this(key, constants, constraints, 0.02);
  }

  @Override
  public void updateTuning(int id, ProfiledPIDController... followers) {
    LoggedTunableNumber.ifChanged(
        id,
        constants -> {
          super.setPID(constants[0], constants[1], constants[2]);
          for (ProfiledPIDController follower : followers) {
            follower.setPID(constants[0], constants[1], constants[2]);
          }
        },
        tunableNumbers);
  }
}
