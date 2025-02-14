// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;

/** Add your docs here. */
public class Autos {
  public static void registerTriggers(Elevator elevator) {
    new EventTrigger("Extend Score").onTrue(elevator.goToPosition(ElevatorPosition.L4));
  }

  public static final Command upCoralAuto() {
    try {

      PathPlannerPath preloadPath = PathPlannerPath.fromChoreoTrajectory("UP Preload - I");
      PathPlannerPath iToStation = PathPlannerPath.fromPathFile("I - UP Far Station");
      PathPlannerPath stationToJ = PathPlannerPath.fromPathFile("I - UP Far Station");
      PathPlannerPath jToStation = PathPlannerPath.fromPathFile("I - UP Far Station");

      return AutoBuilder.followPath(preloadPath).andThen(AutoBuilder.followPath(iToStation));
    } catch (Exception e) {
      @SuppressWarnings("resource")
      Alert alert = new Alert("Failed to load upCoral Auto", AlertType.kError);
      alert.set(true);
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
