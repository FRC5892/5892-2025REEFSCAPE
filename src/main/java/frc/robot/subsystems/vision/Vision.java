// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
      TimeInterpolatableBuffer.createBuffer(1.0);
  @AutoLogOutput private int trigSolverOdometryMissingCount = 0;

  @Setter private boolean useSingleTag = true;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    Logger.recordOutput("Vision/camera0Offset", VisionConstants.robotToCamera0);
    Logger.recordOutput("Vision/camera1Offset", VisionConstants.robotToCamera1);
    Logger.recordOutput("Vision/camera2Offset", VisionConstants.robotToCamera2);

    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<Pose3d> allTrigRobotPoses = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        handlePoseObservation(
            observation, cameraIndex, robotPoses, robotPosesAccepted, robotPosesRejected);
      }
      List<Pose3d> trigRobotPoses = new LinkedList<>();
      if (useSingleTag || true) {

        for (var observation : inputs[cameraIndex].singleTagObservations) {
          if (shouldRejectTagObservation(observation)) {
            continue;
          }
          var opPoseObservation =
              pnpDistanceTrigSolveStrategy(
                  observation,
                  cameraIndex == 0
                      ? robotToCamera0
                      : cameraIndex == 1 ? robotToCamera1 : robotToCamera2);
          if (opPoseObservation.isEmpty()) {
            continue;
          }
          handlePoseObservation(
              opPoseObservation.get(),
              cameraIndex,
              trigRobotPoses,
              robotPosesAccepted,
              robotPosesRejected);
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TrigRobotPoses",
          trigRobotPoses.toArray(new Pose3d[trigRobotPoses.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      allTrigRobotPoses.addAll(trigRobotPoses);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    Logger.recordOutput(
        "Vision/Summary/TrigRobotPoses",
        allTrigRobotPoses.toArray(new Pose3d[allTrigRobotPoses.size()]));
  }

  private Matrix<N3, N1> calculateStdDevs(VisionIO.PoseObservation observation, int cameraIndex) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = linearStdDevBaseline * stdDevFactor;
    double angularStdDev = angularStdDevBaseline * stdDevFactor;
    if (observation.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= linearStdDevMegatag2Factor;
      angularStdDev *= angularStdDevMegatag2Factor;
    } else if (observation.type() == PoseObservationType.PHOTONVISION_TRIG) {
      linearStdDev *= linearStdDevTrigFactor;
      angularStdDev = angularStdDevTrigFactor;
    }
    if (cameraIndex < cameraStdDevFactors.length) {
      linearStdDev *= cameraStdDevFactors[cameraIndex];
      angularStdDev *= cameraStdDevFactors[cameraIndex];
    }
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  private boolean shouldRejectObservation(VisionIO.PoseObservation observation) {
    return observation.tagCount() == 0 // Must have at least one tag
        || (observation.tagCount() == 1
            && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

        // Must be within the field boundaries
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth();
  }

  private Optional<VisionIO.PoseObservation> pnpDistanceTrigSolveStrategy(
      VisionIO.SingleTagObservation observation, Transform3d robotToCamera) {

    var headingSampleOpt = headingBuffer.getSample(observation.timestamp());
    if (headingSampleOpt.isEmpty()) {
      trigSolverOdometryMissingCount++;
      return Optional.empty();
    }
    Rotation2d headingSample = headingSampleOpt.get();
    Translation2d camToTagTranslation =
        new Translation3d(
                observation.cameraToTag().getTranslation().getNorm(),
                new Rotation3d(
                    0, -Math.toRadians(observation.pitch()), -Math.toRadians(observation.yaw())))
            .rotateBy(robotToCamera.getRotation())
            .toTranslation2d()
            .rotateBy(headingSample);

    var tagPoseOpt = aprilTagLayout.getTagPose(observation.id());
    if (tagPoseOpt.isEmpty()) {
      return Optional.empty();
    }
    var tagPose2d = tagPoseOpt.get().toPose2d();

    Translation2d fieldToCameraTranslation =
        tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

    Translation2d camToRobotTranslation =
        robotToCamera.getTranslation().toTranslation2d().unaryMinus().rotateBy(headingSample);

    Pose2d robotPose =
        new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);

    return Optional.of(
        new VisionIO.PoseObservation(
            observation.timestamp(),
            new Pose3d(robotPose),
            -1,
            1,
            camToTagTranslation.getNorm(),
            PoseObservationType.PHOTONVISION_TRIG));
  }

  public void handlePoseObservation(
      VisionIO.PoseObservation observation,
      int cameraIndex,
      List<Pose3d> robotPoses,
      List<Pose3d> robotPosesAccepted,
      List<Pose3d> robotPosesRejected) {
    // Add pose to log
    robotPoses.add(observation.pose());
    if (shouldRejectObservation(observation)) {
      robotPosesRejected.add(observation.pose());
      // Skip if rejected
      return;
    } else {
      robotPosesAccepted.add(observation.pose());
    }
    // Send vision observation
    consumer.accept(
        observation.pose().toPose2d(),
        observation.timestamp(),
        calculateStdDevs(observation, cameraIndex));
  }

  public void consumeYawObservation(double timestamp, Rotation2d yaw) {
    headingBuffer.addSample(timestamp, yaw);
  }

  public boolean shouldRejectTagObservation(VisionIO.SingleTagObservation observation) {
    return observation.cameraToTag().getTranslation().getNorm() > maxRangeTrig;
  }

  public Command setSingleTag(boolean shouldDoSingleTag) {
    return runOnce(() -> this.setSingleTag(shouldDoSingleTag));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
