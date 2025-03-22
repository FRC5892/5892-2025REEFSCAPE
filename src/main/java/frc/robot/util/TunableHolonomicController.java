// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TunableHolonomicController {
  private final TunablePIDController xController;
  private final TunablePIDController yController;
  private final TunablePIDController thetaController;
  @Getter @Setter private Pose2d fieldRelativeSetpoint = new Pose2d();
  private final String key;

  public TunableHolonomicController(
      String key, PIDConstants xConstants, PIDConstants yConstants, PIDConstants thetaConstants) {
    this.key = key;
    xController = new TunablePIDController(key + "/XController", xConstants);
    yController = new TunablePIDController(key + "/YController", yConstants);
    thetaController = new TunablePIDController(key + "/ThetaController", thetaConstants);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds calculateRobotRelative(Transform2d error) {
    Logger.recordOutput(key + "/Error", error);
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
    xController.updateConstants(id);
    yController.updateConstants(id);
    thetaController.updateConstants(id);
  }

  public void reset() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }
}
