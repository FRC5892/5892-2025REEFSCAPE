// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CoralEndEffector.CoralEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {
  private static final LoggedTunableNumber extendDistance =
      new LoggedTunableNumber("Elevator/autoExtendDistanceFromReef", 2.5);

  public static void registerTriggers(Elevator elevator) {
    new EventTrigger("Extend Score").onTrue(elevator.goToPosition(ElevatorPosition.L4));
  }

  public static Command leftCoralAuto(
      Elevator elevatorSubsystem, CoralEndEffector coralSubsystem, Drive drive) {
    try {
      PathPlannerPath pathLeft_Preload_I = PathPlannerPath.fromPathFile("Left Preload - I");
      PathPlannerPath pathI_Left_Far_Station = PathPlannerPath.fromPathFile("I - Left Far Station");
      PathPlannerPath pathLeft_Far_Station_J = PathPlannerPath.fromPathFile("Left Far Station - J");
      PathPlannerPath pathJ_Left_Far_Station = PathPlannerPath.fromPathFile("J - Left Far Station");
      PathPlannerPath pathLeft_Far_Station_K = PathPlannerPath.fromPathFile("Left Far Station - K");
      PathPlannerPath pathK_Left_Far_Station = PathPlannerPath.fromPathFile("K - Left Far Station");
      PathPlannerPath pathLeft_Far_Station_L = PathPlannerPath.fromPathFile("Left Far Station - L");

      if (Constants.currentMode == Constants.Mode.SIM) {
        var points = new ArrayList<>(pathLeft_Preload_I.getAllPathPoints());
        points.addAll(pathI_Left_Far_Station.getAllPathPoints());
        points.addAll(pathLeft_Far_Station_J.getAllPathPoints());
        points.addAll(pathJ_Left_Far_Station.getAllPathPoints());
        points.addAll(pathLeft_Far_Station_K.getAllPathPoints());
        points.addAll(pathK_Left_Far_Station.getAllPathPoints());
        points.addAll(pathLeft_Far_Station_L.getAllPathPoints());
        Logger.recordOutput(
            "Autos/LeftCoralAuto/Path",
            points.stream().map((p) -> p.position).toArray(Translation2d[]::new));
      }

      return AutoBuilder.pathfindThenFollowPath(
              pathLeft_Preload_I, new PathConstraints(10, 10, 10, 10))
          .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.L4))
          .andThen(
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathI_Left_Far_Station)
                  .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathLeft_Far_Station_J)
                  .alongWith(
                      intakeThenExtend(
                          elevatorSubsystem, coralSubsystem, drive, ElevatorPosition.L4)),
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathJ_Left_Far_Station)
                  .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathLeft_Far_Station_K)
                  .alongWith(
                      intakeThenExtend(
                          elevatorSubsystem, coralSubsystem, drive, ElevatorPosition.L4)),
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathK_Left_Far_Station)
                  .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE)),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathLeft_Far_Station_L)
                  .alongWith(
                      intakeThenExtend(
                          elevatorSubsystem, coralSubsystem, drive, ElevatorPosition.L4)),
              outtakeCoral(coralSubsystem),
              elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE));
    } catch (Exception e) {
      @SuppressWarnings("resource")
      Alert alert = new Alert("Failed to load upCoral Auto", AlertType.kError);
      alert.set(true);
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command waitForCoral(CoralEndEffector coralSubsystem) {
    return Commands.waitUntil(coralSubsystem::getBeamBreak);
  }

  public static Command outtakeCoral(CoralEndEffector coralSubsystem) {
    return coralSubsystem.runOuttake().withTimeout(0.25);
  }

  public static Command intakeThenExtend(
      Elevator elevatorSubsystem,
      CoralEndEffector coralSubsystem,
      Drive drive,
      ElevatorPosition position) {
    return coralSubsystem
        .runIntake()
        .until(() -> !coralSubsystem.getBeamBreak())
        .andThen(
            Commands.waitUntil(() -> drive.getDistanceToReefM() < extendDistance.get()),
            elevatorSubsystem.goToPosition(position));
  }
}
