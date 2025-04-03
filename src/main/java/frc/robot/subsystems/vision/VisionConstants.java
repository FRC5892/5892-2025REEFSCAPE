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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout;

  static {
    try {
      aprilTagLayout =
          new AprilTagFieldLayout(
              Path.of(
                  Filesystem.getDeployDirectory().getAbsolutePath(),
                  "2025-reefscape-andymark.json"));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "leftFront";
  public static String camera1Name = "rightFront";
  public static String camera2Name = "centerFront";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Inches.of(12.625),
          Inches.of(9.75),
          Inches.of(12.5),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-54.5)));
  public static Transform3d robotToCamera1 = // 10 7/8 12.5
      new Transform3d(
          Inches.of(12.625),
          Inches.of(-9.75),
          Inches.of(12.5),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(55)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Inches.of(15),
          Inches.of(-1.5),
          Inches.of(8.875),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0 // Camera 2
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

  // No rotation data available
  public static double rangeMultiplierTrigFactor = 2; // Not better at high range
  public static double linearStdDevTrigFactor = 0.10; // Very useful for close range
  public static double angularStdDevTrigFactor =
      Double.POSITIVE_INFINITY; // It literally uses rotation data
  public static double maxRangeTrig = 1.5; // Meters
}
