// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TunableProfiledPIDController extends ProfiledPIDController
/*implements Tunable<ProfiledPIDController>*/ {
  private final LoggedTunableNumber[] tunableNumbers;
  private final String key;

  public TunableProfiledPIDController(
      String key,
      PIDConstants constants,
      TrapezoidProfile.Constraints constraints,
      double positionTolerance,
      double period) {
    super(constants.kP, constants.kI, constants.kD, constraints, period);
    this.key = key;
    super.setIntegratorRange(-constants.iZone, constants.iZone);
    tunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(key + "/kP", constants.kP),
          new LoggedTunableNumber(key + "/kI", constants.kI),
          new LoggedTunableNumber(key + "/kD", constants.kD),
          new LoggedTunableNumber(key + "/maxVelocity", constraints.maxVelocity),
          new LoggedTunableNumber(key + "/maxAccel", constraints.maxAcceleration),
          new LoggedTunableNumber(key + "/positionTolerance", positionTolerance)
        };
  }

  public TunableProfiledPIDController(
      String key,
      PIDConstants constants,
      TrapezoidProfile.Constraints constraints,
      double positionTolerance) {
    this(key, constants, constraints, positionTolerance, 0.02);
  }

  public double calculate(double measurement) {
    double result = super.calculate(measurement);
    Logger.recordOutput(key + "/result", result);
    return result;
  }

  public void updateTuning(Object id, ProfiledPIDController... followers) {
    LoggedTunableNumber.ifChanged(
        id,
        constants -> {
          super.setPID(constants[0], constants[1], constants[2]);
          super.setConstraints(new TrapezoidProfile.Constraints(constants[3], constants[4]));
          super.setTolerance(constants[5]);

          for (ProfiledPIDController follower : followers) {
            follower.setPID(constants[0], constants[1], constants[2]);
            follower.setConstraints(new TrapezoidProfile.Constraints(constants[3], constants[4]));
            follower.setTolerance(constants[5]);
          }
        },
        tunableNumbers);
  }
}
