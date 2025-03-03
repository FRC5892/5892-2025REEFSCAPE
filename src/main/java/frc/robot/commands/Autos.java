// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CoralEndEffector.CoralEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {
  private static final LoggedTunableNumber extendDistance =
      new LoggedTunableNumber("Autos/autoExtendDistanceFromReef", 4);
  private static final PathConstraints constraints = new PathConstraints(10, 10, 10, 10);

  public static SendableChooser<Command> buildAutoChooser(
      Elevator elevatorSubsystem, CoralEndEffector coralSubsystem, Drive drive) {
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Left Auto", leftCoralAuto(elevatorSubsystem, coralSubsystem, drive));
    // chooser.addOption("Right Auto", rightCoralAuto(elevatorSubsystem, coralSubsystem, drive));
    // chooser.addOption(
    // "Center Right Auto", centerRightPreload(elevatorSubsystem, coralSubsystem, drive));
    return chooser;
  }

  public static Command leftCoralAuto(
      Elevator elevatorSubsystem, CoralEndEffector coralSubsystem, Drive drive) {
    try {
      final ArrayList<PathPoint> points;
      if (Constants.currentMode == Constants.Mode.SIM) {
        points = new ArrayList<>();
      } else {
        points = null;
      }
      final Command auto =
          AutoBuilder.followPath(loadPath("Left Preload - I", points))
              .alongWith(extendAtPosition(elevatorSubsystem, drive, ElevatorPosition.L4))
              .andThen(
                  outtakeCoral(coralSubsystem),
                  loadLogFollow("I - Left Far Station", points)
                      .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
                  intake(coralSubsystem),
                  loadLogFollow("Left Far Station - K", points)
                      .alongWith(extendAtPosition(elevatorSubsystem, drive, ElevatorPosition.L4)),
                  outtakeCoral(coralSubsystem),
                  loadLogFollow("K - Left Far Station", points)
                      .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
                  intake(coralSubsystem),
                  loadLogFollow("Left Far Station - L", points)
                      .alongWith(extendAtPosition(elevatorSubsystem, drive, ElevatorPosition.L4)),
                  outtakeCoral(coralSubsystem),
                  loadLogFollow("L - Left Far Station", points)
                      .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
                  intake(coralSubsystem),
                  loadLogFollow("Left Far Station - J", points)
                      .alongWith(extendAtPosition(elevatorSubsystem, drive, ElevatorPosition.L4)),
                  outtakeCoral(coralSubsystem),
                  elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE));
      if (Constants.currentMode == Constants.Mode.SIM) {
        Logger.recordOutput(
            "Autos/Left Auto", points.stream().map(m -> m.position).toArray(Translation2d[]::new));
      }
      return auto;
    } catch (Exception e) {
      @SuppressWarnings("resource")
      Alert alert = new Alert("Failed to load leftCoral Auto", AlertType.kError);
      alert.set(true);
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // public static Command rightCoralAuto(
  //     Elevator elevatorSubsystem, CoralEndEffector coralSubsystem, Drive drive) {
  //   try {
  //     final ArrayList<PathPoint> points;
  //     if (Constants.currentMode == Constants.Mode.SIM) {
  //       points = new ArrayList<>();
  //     } else {
  //       points = null;
  //     }
  //     final Command auto =
  //         AutoBuilder.pathfindThenFollowPath(loadPath("Right Preload - F", points), constraints)
  //             .andThen(elevatorSubsystem.goToPosition(ElevatorPosition.L4))
  //             .andThen(
  //                 outtakeCoral(coralSubsystem),
  //                 loadLogFollow("F - Right Far Station", points)
  //                     .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
  //                 waitForCoral(coralSubsystem),
  //                 loadLogFollow("Right Far Station - E", points)
  //                     .alongWith(
  //                         intakeThenExtend(
  //                             elevatorSubsystem, coralSubsystem, drive, ElevatorPosition.L4)),
  //                 outtakeCoral(coralSubsystem),
  //                 loadLogFollow("E - Right Far Station", points)
  //                     .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
  //                 waitForCoral(coralSubsystem),
  //                 loadLogFollow("Right Far Station - D", points)
  //                     .alongWith(
  //                         intakeThenExtend(
  //                             elevatorSubsystem, coralSubsystem, drive, ElevatorPosition.L4)),
  //                 outtakeCoral(coralSubsystem),
  //                 loadLogFollow("D - Right Far Station", points)
  //                     .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
  //                 waitForCoral(coralSubsystem),
  //                 loadLogFollow("Right Far Station - C", points)
  //                     .alongWith(
  //                         intakeThenExtend(
  //                             elevatorSubsystem, coralSubsystem, drive, ElevatorPosition.L4)),
  //                 outtakeCoral(coralSubsystem),
  //                 elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE));
  //     if (Constants.currentMode == Constants.Mode.SIM) {
  //       Logger.recordOutput(
  //           "Autos/Right Auto", points.stream().map(m ->
  // m.position).toArray(Translation2d[]::new));
  //     }
  //     return auto;
  //   } catch (Exception e) {
  //     @SuppressWarnings("resource")
  //     Alert alert = new Alert("Failed to load rightCoral Auto", AlertType.kError);
  //     alert.set(true);
  //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  //     return Commands.none();
  //   }
  // }

  public static Command elevatorExtendAuto(Elevator elevatorSub) {
    return elevatorSub.goToPosition(ElevatorPosition.L4);
  }

  // public static Command centerRightPreload(
  //     Elevator elevatorSubsystem, CoralEndEffector coralSubsystem, Drive drive) {
  //   try {
  //     final ArrayList<PathPoint> points;
  //     if (Constants.currentMode == Constants.Mode.SIM) {
  //       points = new ArrayList<>();
  //     } else {
  //       points = null;
  //     }
  //     final Command auto =
  //         AutoBuilder.pathfindThenFollowPath(loadPath("Center Preload - G", points), constraints)
  //             .alongWith(
  //                 intakeThenExtend(elevatorSubsystem, coralSubsystem, drive,
  // ElevatorPosition.L4))
  //             .andThen(
  //                 intakeThenExtend(elevatorSubsystem, coralSubsystem, drive,
  // ElevatorPosition.L4),
  //                 outtakeCoral(coralSubsystem),
  //                 elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE));
  //     if (Constants.currentMode == Constants.Mode.SIM) {
  //       Logger.recordOutput(
  //           "Autos/Center Right Auto",
  //           points.stream().map(m -> m.position).toArray(Translation2d[]::new));
  //     }
  //     return auto;
  //   } catch (Exception e) {
  //     @SuppressWarnings("resource")
  //     Alert alert = new Alert("Failed to load rightCoral Auto", AlertType.kError);
  //     alert.set(true);
  //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  //     return Commands.none();
  //   }
  // }

  public static Command waitForCoral(CoralEndEffector coralSubsystem) {
    return Commands.waitUntil(coralSubsystem::getBeamBreak);
  }

  public static Command outtakeCoral(CoralEndEffector coralSubsystem) {
    return coralSubsystem.runOuttake().withTimeout(0.5);
  }

  // public static Command intakeThenExtend(
  //     Elevator elevatorSubsystem,
  //     CoralEndEffector coralSubsystem,
  //     Drive drive,
  //     ElevatorPosition position) {
  //   return coralSubsystem
  //       .runIntake()
  //       .until(() -> !coralSubsystem.isDebouncedBeamBreak())
  //       .andThen(
  //          Commands.waitUntil(() -> drive.getDistanceToReefM() < extendDistance.get()),
  //          elevatorSubsystem.goToPosition(position)
  //       );
  // }
  public static Command extendAtPosition(
      Elevator elevator, Drive drive, ElevatorPosition position) {
    return Commands.waitUntil(() -> drive.getDistanceToReefM() < extendDistance.get())
        .andThen(elevator.goToPosition(position));
  }

  public static Command intake(CoralEndEffector coralSubsystem) {
    return Commands.waitUntil(coralSubsystem::isDebouncedBeamBreak)
        .andThen(coralSubsystem.runIntake().until(() -> !coralSubsystem.isDebouncedBeamBreak()));
  }

  public static PathPlannerPath loadPath(String name, List<PathPoint> points)
      throws IOException, ParseException {
    var path = PathPlannerPath.fromPathFile(name);
    if (Constants.currentMode == Constants.Mode.SIM) {
      points.addAll(path.getAllPathPoints());
    }
    return path;
  }

  public static Command loadLogFollow(String name, List<PathPoint> points)
      throws IOException, ParseException {
    return AutoBuilder.followPath(loadPath(name, points))
        .withName(name)
        .alongWith(Commands.print("Following path " + name));
  }
}
