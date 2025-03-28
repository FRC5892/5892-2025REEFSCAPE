// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TunableHolonomicController {
  private final TunableProfiledPIDController xController;
  private final TunableProfiledPIDController yController;
  private final TunableProfiledPIDController thetaController;
  @Getter @Setter private Pose2d fieldRelativeSetpoint = new Pose2d();
  private final String key;

  public TunableHolonomicController(
      String key,
      PIDConstants xConstants,
      Constraints xConstraints,
      double xTolerance,
      PIDConstants yConstants,
      Constraints yConstraints,
      double yTolerance,
      PIDConstants thetaConstants,
      Constraints thetaConstraints,
      double thetaTolerance) {
    this.key = key;
    xController =
        new TunableProfiledPIDController(
            key + "/XController", xConstants, xConstraints, xTolerance);
    yController =
        new TunableProfiledPIDController(
            key + "/YController", yConstants, yConstraints, yTolerance);
    thetaController =
        new TunableProfiledPIDController(
            key + "/ThetaController", thetaConstants, thetaConstraints, thetaTolerance);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds calculateRobotRelative(Transform2d error) {
    Logger.recordOutput(key + "/Error", error);
    Logger.recordOutput("key", xController.getVelocityError());
    return new ChassisSpeeds(
        // We already have error, so skip its internal subtraction by setting the measurement to 0
        xController.calculate(0, error.getTranslation().getX()),
        yController.calculate(0, error.getTranslation().getY()),
        thetaController.calculate(0, error.getRotation().getRadians()));
  }

  public ChassisSpeeds calculateFieldRelative(Pose2d currentPose) {
    return calculateRobotRelative(fieldRelativeSetpoint.minus(currentPose));
  }

  public ChassisSpeeds calculateFieldRelative(Pose2d currentPose, Pose2d targetPose) {
    fieldRelativeSetpoint = targetPose;
    return calculateFieldRelative(currentPose);
  }

  public void updateConstants(Object id) {
    xController.updateTuning(id);
    yController.updateTuning(id);
    thetaController.updateTuning(id);
  }

  public void reset(ChassisSpeeds speeds) {
    xController.reset(0, speeds.vxMetersPerSecond);
    yController.reset(0, speeds.vyMetersPerSecond);
    thetaController.reset(0, speeds.omegaRadiansPerSecond);
  }

  public boolean atSetpoint() {
    return xController.atGoal() && yController.atGoal() & thetaController.atGoal();
  }
}
