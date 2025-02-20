// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralEndEffector.CoralEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;

/** Add your docs here. */
public class Autos {
  public static void registerTriggers(Elevator elevator) {
    new EventTrigger("Extend Score").onTrue(elevator.goToPosition(ElevatorPosition.L4));
  }

  public static Command leftCoralAuto(Elevator elevatorSubsystem, CoralEndEffector coralSubsystem) {
    try {
      PathPlannerPath pathLPreI = PathPlannerPath.fromChoreoTrajectory("LPreI");
      PathPlannerPath pathIS = PathPlannerPath.fromChoreoTrajectory("IS");
      PathPlannerPath pathSJ = PathPlannerPath.fromChoreoTrajectory("SJ");
      PathPlannerPath pathJS = PathPlannerPath.fromChoreoTrajectory("JS");
      PathPlannerPath pathSK = PathPlannerPath.fromChoreoTrajectory("SK");
      PathPlannerPath pathKS = PathPlannerPath.fromChoreoTrajectory("KS");
      PathPlannerPath pathSL = PathPlannerPath.fromChoreoTrajectory("SL");
      PathPlannerPath pathLS = PathPlannerPath.fromChoreoTrajectory("LS");
      PathPlannerPath pathSA = PathPlannerPath.fromChoreoTrajectory("SA");
      return AutoBuilder.pathfindThenFollowPath(pathLPreI, new PathConstraints(10, 10, 10, 10))
          .alongWith(elevatorSubsystem.goToPosition(ElevatorPosition.L4))
          .andThen(
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathIS)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE),
                      Commands.print("pathIS")),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathSJ)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.L4),
                      Commands.print("pathSJ")),
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathJS)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE),
                      Commands.print("pathJS")),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathSK)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.L4),
                      Commands.print("pathSK")),
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathKS)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE),
                      Commands.print("pathKS")),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathSL)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.L4),
                      Commands.print("pathSL")),
              outtakeCoral(coralSubsystem),
              AutoBuilder.followPath(pathLS)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.INTAKE),
                      Commands.print("pathLS")),
              waitForCoral(coralSubsystem),
              AutoBuilder.followPath(pathSA)
                  .alongWith(
                      elevatorSubsystem.goToPosition(ElevatorPosition.L4),
                      Commands.print("pathSA")),
              outtakeCoral(coralSubsystem));
    } catch (Exception e) {
      @SuppressWarnings("resource")
      Alert alert = new Alert("Failed to load upCoral Auto", AlertType.kError);
      alert.set(true);
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command waitForCoral(CoralEndEffector coralSubsystem) {
    return Commands.waitUntil(
        () -> {
          return coralSubsystem.getBeamBreak();
        });
  }

  public static Command outtakeCoral(CoralEndEffector coralSubsystem) {
    return coralSubsystem.runOuttake().withTimeout(0.25).asProxy();
  }
}
