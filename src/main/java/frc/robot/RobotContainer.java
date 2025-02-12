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

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.CoralEndEffector.CoralEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator.ElevatorSimulation;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedDIO.NoOppDio;
import frc.robot.util.LoggedDIO.SimDIO;
import frc.robot.util.LoggedTalon.NoOppTalonFX;
import frc.robot.util.LoggedTalon.PhoenixTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CANBus defaultCanBus = new CANBus("rio");
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Climb climb;
  private final CoralEndEffector coralEndEffector;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController codriverController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case DEV:
        // Disabled IO for everything but the tested subsystem
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        // coralEndEffector =
        //     new CoralEndEffector(
        //         new NoOppTalonFX("coralEffector", 0), new NoOppDio("intakeBeamBreak"));
        climb = new Climb(new NoOppTalonFX("climb", 0));

        // Instantiate the tested subsystem

        elevator = new Elevator(new PhoenixTalonFX(20, defaultCanBus, "elevator"));
        coralEndEffector =
            new CoralEndEffector(
                new PhoenixTalonFX(22, defaultCanBus, "coralEffector"),
                new HardwareDIO("intakeBeamBreak", 0));

        break;
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        elevator = new Elevator(new PhoenixTalonFX(20, defaultCanBus, "elevator"));
        coralEndEffector =
            new CoralEndEffector(
                new PhoenixTalonFX(-2, defaultCanBus, "coralEffector"),
                new HardwareDIO("intakeBeamBreak", 2));
        climb = new Climb(new PhoenixTalonFX(-3, defaultCanBus, "climb"));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        elevator = new Elevator(new ElevatorSimulation(5, defaultCanBus, "elevator"));
        coralEndEffector =
            new CoralEndEffector(
                new PhoenixTalonFX(-2, defaultCanBus, "coralEffector"),
                SimDIO.fromNT("intakeBeamBreak"));
        climb = new Climb(new PhoenixTalonFX(-3, defaultCanBus, "climb"));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        elevator = new Elevator(new NoOppTalonFX("elevator", 0));
        coralEndEffector =
            new CoralEndEffector(
                new NoOppTalonFX("coralEffector", 0), new NoOppDio("intakeBeamBreak"));
        climb = new Climb(new NoOppTalonFX("climb", 0));
        break;
    }

    coralEndEffector
        .beamBreakTrigger()
        .and(() -> elevator.atPosition(ElevatorPosition.INTAKE))
        .whileTrue(
            coralEndEffector
                .runIntake()
                .andThen(rumbleBoth(GenericHID.RumbleType.kLeftRumble, 1, 0.25))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Controls */

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX()));
    // Reset gyro to 0° when B button is pressed
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driverController.leftBumper().whileTrue(drive.driveToReefCommand(Drive.ReefBranch.LEFT));
    driverController.rightBumper().whileTrue(drive.driveToReefCommand(Drive.ReefBranch.RIGHT));
    /* Codriver Controls */
    driverController.a().onTrue(elevator.goToPosition(ElevatorPosition.INTAKE));
    driverController.b().onTrue(elevator.goToPosition(ElevatorPosition.L2));
    driverController.x().onTrue(elevator.goToPosition(ElevatorPosition.L3));
    driverController.y().onTrue(elevator.goToPosition(ElevatorPosition.L4));
    driverController.start().onTrue(elevator.set0());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command rumbleDriver(
      GenericHID.RumbleType rumbleType, double intensity, double timeSeconds) {
    return Commands.runOnce(
            () -> {
              driverController.setRumble(rumbleType, intensity);
            })
        .andThen(new WaitCommand(timeSeconds))
        .finallyDo(() -> driverController.setRumble(rumbleType, 0));
  }

  public Command rumbleCoDriver(
      GenericHID.RumbleType rumbleType, double intensity, double timeSeconds) {
    return Commands.runOnce(
            () -> {
              codriverController.setRumble(rumbleType, intensity);
            })
        .andThen(new WaitCommand(timeSeconds))
        .finallyDo(() -> codriverController.setRumble(rumbleType, 0));
  }

  public Command rumbleBoth(
      GenericHID.RumbleType rumbleType, double intensity, double timeSeconds) {
    return Commands.parallel(
        rumbleDriver(rumbleType, intensity, timeSeconds),
        rumbleCoDriver(rumbleType, intensity, timeSeconds));
  }
}
