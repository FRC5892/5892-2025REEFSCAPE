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
import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.CoralEndEffector.CoralEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator.ElevatorSimulation;
import frc.robot.subsystems.LEDS.Leds;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelServoHub;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedDIO.NoOppDio;
import frc.robot.util.LoggedDIO.SimDIO;
import frc.robot.util.LoggedServo.NoOppServo;
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
  private final Funnel funnel;
  private final StringPublisher selectedDashboardTabPublisher =
      NetworkTableInstance.getDefault()
          .getStringTopic("/Elastic/SelectedTab")
          .publish(PubSubOption.keepDuplicates(true));
  //   private final AlgaeRemover algaeRemover;
  //   private final BatteryTracking batteryTracking;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController codriverController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      //   case DEV:
      //     // Disabled IO for everything but the tested subsystem
      //     // drive =
      //     //     new Drive(
      //     //         new GyroIO() {},
      //     //         new ModuleIO() {},
      //     //         new ModuleIO() {},
      //     //         new ModuleIO() {},
      //     //         new ModuleIO() {});
      //     drive =
      //         new Drive(
      //             new GyroIOPigeon2(),
      //             new ModuleIOTalonFX(TunerConstants.FrontLeft),
      //             new ModuleIOTalonFX(TunerConstants.FrontRight),
      //             new ModuleIOTalonFX(TunerConstants.BackLeft),
      //             new ModuleIOTalonFX(TunerConstants.BackRight));
      //     vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
      //     // coralEndEffector =
      //     //     new CoralEndEffector(
      //     //         new NoOppTalonFX("coralEffector", 0), new NoOppDio("intakeBeamBreak"));
      //     climb =
      //         new Climb(
      //             new PhoenixTalonFX(-3, defaultCanBus, "climb"),
      //             new HardwareDIO("climbForwardLimit", 1),
      //             new HardwareDIO("climbReverseLimit", 2));
      //     // Instantiate the tested subsystem

      //     elevator = new Elevator(new PhoenixTalonFX(20, defaultCanBus, "elevator"));
      //     coralEndEffector =
      //         new CoralEndEffector(
      //             new PhoenixTalonFX(22, defaultCanBus, "coralEffector"),
      //             new HardwareDIO("intakeBeamBreak", 0));
      //     funnel = new Funnel(new FunnelServoHub(-1, 500, 2500).getServo(ChannelId.kChannelId0));

      //     break;
      case REAL:
        // Real robot, instantiate hardware IO implementations
        final FunnelServoHub servoHub = new FunnelServoHub(30, 500, 2500);
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
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2));
        elevator = new Elevator(new PhoenixTalonFX(20, defaultCanBus, "elevator"));
        coralEndEffector =
            new CoralEndEffector(
                new PhoenixTalonFX(22, defaultCanBus, "coralEffector"),
                new HardwareDIO("intakeBeamBreak", 0));
        funnel = new Funnel(servoHub.getServo(ChannelId.kChannelId0));
        climb =
            new Climb(
                new PhoenixTalonFX(23, defaultCanBus, "climb"),
                new HardwareDIO("climbForwardLimit", 1),
                new HardwareDIO("climbReverseLimit", 2));
        new Leds();
        // algaeRemover =
        //     new AlgaeRemover(
        //         // servoHub.getServo(ChannelId.kChannelId1),
        //         new PhoenixTalonFX(24, defaultCanBus, "algaeRemover"));
        // batteryTracking = new BatteryTracking(new BatteryTrackingReal());
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
                //                new VisionConsumer() {
                //                  public void accept(
                //                      Pose2d visionRobotPoseMeters,
                //                      double timestampSeconds,
                //                      Matrix<N3, N1> visionMeasurementStdDevs) {}
                //                },
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose));
        elevator = new Elevator(new ElevatorSimulation(20, defaultCanBus, "elevator"));
        coralEndEffector =
            new CoralEndEffector(
                new PhoenixTalonFX(22, defaultCanBus, "coralEffector"),
                SimDIO.fromNT("intakeBeamBreak"));
        climb =
            new Climb(
                new PhoenixTalonFX(23, defaultCanBus, "climb"),
                SimDIO.fromNT("climbForwardLimit"),
                SimDIO.fromNT("climbReverseLimit"));
        funnel = new Funnel(new NoOppServo(500, 2500));
        // algaeRemover =
        //     new AlgaeRemover(
        //         /*new NoOppServo(500, 2500),*/ new SimpleMotorSim(
        //             24, defaultCanBus, "algaeRemover", 2, 1));
        // batteryTracking = new BatteryTracking(new BatteryTrackingReal(), () -> Math.random() *
        // 40);

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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        elevator = new Elevator(new NoOppTalonFX("elevator", 0));
        coralEndEffector =
            new CoralEndEffector(
                new NoOppTalonFX("coralEffector", 0), new NoOppDio("intakeBeamBreak"));
        climb =
            new Climb(
                new NoOppTalonFX("climb", 0),
                new NoOppDio("climbForwardLimit"),
                new NoOppDio("climbReverseLimit"));
        funnel = new Funnel(new NoOppServo(500, 2500));
        // algaeRemover =
        //     new AlgaeRemover(/*new NoOppServo(500, 2500),*/ new NoOppTalonFX("algaeRemover", 0));
        // batteryTracking = new BatteryTracking(new BatteryTrackingNoOpp() {}, () -> 0);

        break;
    }
    drive.registerYawConsumer(vision::consumeYawObservation);

    // coralEndEffector
    //     .beamBreakTrigger()
    //     .and(() -> elevator.atPosition(ElevatorPosition.INTAKE) &&
    // DriverStation.isTeleopEnabled())
    //     .whileTrue(
    //         coralEndEffector
    //             .runIntake()
    //             .andThen(rumbleBoth(GenericHID.RumbleType.kLeftRumble, 1, 0.25))
    //             .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    // RobotModeTriggers.teleop().onFalse(batteryTracking.writeCommand());
    // RobotModeTriggers.autonomous()
    //     .and(() -> !DriverStation.isFMSAttached())
    //     .onFalse(batteryTracking.writeCommand());
    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices", Autos.buildAutoChooser(elevator, coralEndEffector, drive, vision));

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
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
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
    /* Robot Controls */
    coralEndEffector
        .beamBreakTrigger()
        .and(() -> elevator.atPosition(ElevatorPosition.INTAKE))
        .and(DriverStation::isTeleop)
        .whileTrue(
            coralEndEffector
                .runIntake()
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))
        .onFalse(rumbleBoth(GenericHID.RumbleType.kBothRumble, 1, 0.02));

    // if (powerDistribution != null) {
    //   driverController
    //       .back()
    //       .onTrue(
    //           Commands.runOnce(
    //               () ->
    //                   powerDistribution.setSwitchableChannel(
    //                       powerDistribution.getSwitchableChannel())));
    // }
    /* Driver Controls */
    DriverStation.silenceJoystickConnectionWarning(true);

    RobotModeTriggers.autonomous()
        .or(DriverStation::isTeleopEnabled)
        .onTrue(funnel.move(Funnel.FunnelPosition.DOWN));
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    // Reset gyro to 0° when B button is pressed
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))),
                    drive)
                .ignoringDisable(true));
    driverController
        .leftBumper()
        .whileTrue(
            drive
                .alignToReefCommand(Drive.ReefBranch.LEFT)
                .andThen(rumbleBoth(RumbleType.kBothRumble, 1, 0.25)));
    driverController
        .rightBumper()
        .whileTrue(
            drive
                .alignToReefCommand(Drive.ReefBranch.RIGHT)
                .andThen(rumbleBoth(RumbleType.kBothRumble, 1, 0.25)));
    driverController.povUp().whileTrue(drive.nudgeCommand(Drive.NudgeDirection.FORWARD));
    driverController.povDown().whileTrue(drive.nudgeCommand(Drive.NudgeDirection.BACKWARD));
    driverController.povLeft().whileTrue(drive.nudgeCommand(Drive.NudgeDirection.LEFT));
    driverController.povRight().whileTrue(drive.nudgeCommand(Drive.NudgeDirection.RIGHT));

    /* Codriver Controls */
    //    driverController.a().onTrue(elevator.goToPosition(ElevatorPosition.L1));
    codriverController.b().onTrue(elevator.goToPosition(ElevatorPosition.L2));
    codriverController.x().onTrue(elevator.goToPosition(ElevatorPosition.L3));
    codriverController.y().onTrue(elevator.goToPosition(ElevatorPosition.L4));

    codriverController.rightBumper().onTrue(elevator.goToPosition(ElevatorPosition.INTAKE));
    codriverController.start().whileTrue(elevator.homeCommand());

    codriverController.leftBumper().whileTrue(coralEndEffector.runOuttake());

    codriverController
        .povUp()
        .whileTrue(
            funnel
                .move(Funnel.FunnelPosition.UP)
                .alongWith(climb.climbExtend())
                .alongWith(Commands.runOnce(() -> selectedDashboardTabPublisher.set("Climb"))));
    codriverController
        .povRight()
        .onTrue(
            funnel
                .move(Funnel.FunnelPosition.DOWN)
                .alongWith(
                    Commands.runOnce(() -> selectedDashboardTabPublisher.set("Teleoperated"))));
    codriverController.povDown().whileTrue(climb.climbRetract());
    codriverController.povLeft().onTrue(funnel.move(Funnel.FunnelPosition.STARTING));
    // if (Constants.tuningMode) {
    //   codriverController
    //       .rightTrigger(0.25)
    //       .whileTrue(Autos.intakeShoot(elevator, coralEndEffector));
    // }
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
    return Commands.runEnd(
            () -> {
              driverController.setRumble(rumbleType, intensity);
            },
            () -> driverController.setRumble(rumbleType, 0))
        .withTimeout(timeSeconds);
  }

  public Command rumbleCoDriver(
      GenericHID.RumbleType rumbleType, double intensity, double timeSeconds) {
    return Commands.runEnd(
            () -> {
              codriverController.setRumble(rumbleType, intensity);
            },
            () -> codriverController.setRumble(rumbleType, 0))
        .withTimeout(timeSeconds);
  }

  public Command rumbleBoth(
      GenericHID.RumbleType rumbleType, double intensity, double timeSeconds) {
    return Commands.parallel(
        rumbleDriver(rumbleType, intensity, timeSeconds),
        rumbleCoDriver(rumbleType, intensity, timeSeconds));
  }
}
