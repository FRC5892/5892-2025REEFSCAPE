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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  // 5892
  private final LoggedTunableNumber driveKPTunableNumber;
  private final LoggedTunableNumber driveKITunableNumber;
  private final LoggedTunableNumber driveKDTunableNumber;
  private final LoggedTunableNumber driveKSTunableNumber;
  private final LoggedTunableNumber driveKVTunableNumber;

  @Getter @AutoLogOutput private int reefSector = -1;
  @Getter @AutoLogOutput private double distanceToReefM = -1;

  private BiConsumer<Double, Rotation2d> yawConsumer = null;

  // End 5892

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            Constants.currentMode == Mode.SIM
                ? new PIDConstants(5, 0.0, 0.0)
                : new PIDConstants(1.5, 0.0, 0.0),
            Constants.currentMode == Mode.SIM
                ? new PIDConstants(5, 0.0, 0.0)
                : new PIDConstants(1.5, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    // 5892
    if (Constants.tuningMode) {
      if (Constants.currentMode == Mode.SIM) {
        driveKPTunableNumber = new LoggedTunableNumber("Drive kP", ModuleIOSim.DRIVE_KP);
        driveKITunableNumber = new LoggedTunableNumber("Drive kI", 0.0);
        driveKDTunableNumber = new LoggedTunableNumber("Drive kD", ModuleIOSim.DRIVE_KD);
        driveKSTunableNumber = new LoggedTunableNumber("Drive kS", ModuleIOSim.DRIVE_KS);
        driveKVTunableNumber = new LoggedTunableNumber("Drive kV", ModuleIOSim.DRIVE_KV);
      } else {
        driveKPTunableNumber =
            new LoggedTunableNumber("Drive kP", TunerConstants.BackLeft.DriveMotorGains.kP);
        driveKITunableNumber =
            new LoggedTunableNumber("Drive kI", TunerConstants.BackLeft.DriveMotorGains.kI);
        driveKDTunableNumber =
            new LoggedTunableNumber("Drive kD", TunerConstants.BackLeft.DriveMotorGains.kD);
        driveKSTunableNumber =
            new LoggedTunableNumber("Drive kS", TunerConstants.BackLeft.DriveMotorGains.kS);
        driveKVTunableNumber =
            new LoggedTunableNumber("Drive kV", TunerConstants.BackLeft.DriveMotorGains.kV);
      }
    } else {
      driveKPTunableNumber = null;
      driveKITunableNumber = null;
      driveKDTunableNumber = null;
      driveKSTunableNumber = null;
      driveKVTunableNumber = null;
    }
    // End 5892
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      Pose2d pose =
          poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      yawConsumer.accept(sampleTimestamps[i], pose.getRotation());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // 5892
    LoggedTunableNumber.ifChanged(
        this,
        this::updateDrivePID,
        driveKPTunableNumber,
        driveKITunableNumber,
        driveKDTunableNumber,
        driveKSTunableNumber,
        driveKVTunableNumber);
    Translation2d robotToReef =
        getPose().getTranslation().minus(AllianceFlipUtil.apply(FieldConstants.Reef.center));
    double angleOffset =
        (AllianceFlipUtil.shouldFlip() ? 1.0 : 0.5) + (double) 1 / 12 /* add a half sector*/;
    reefSector =
        (int)
                    ((robotToReef.getAngle().getRotations() + angleOffset)
                        * 6 /* 6 sides, so on a scale of 0-6 */)
                % 6 /*Wrap around if it's more than 6 */
            + 1 /*Driver count from one :( */;

    distanceToReefM = robotToReef.getNorm();

    Logger.recordOutput(
        "Drive/ReefSectorSide",
        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[reefSector - 1]));

    // End 5892
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  // 5892
  private void updateDrivePID(double[] values) {
    double kP = values[0];
    double kI = values[1];
    double kD = values[2];
    double kS = values[3];
    double kV = values[4];
    for (int i = 0; i < 4; i++) {
      modules[i].setDrivePID(kP, kI, kD, kS, kV);
    }
  }

  private final double FRONT_TO_CENTER_METERS = 0.47;

  private final LoggedTunableNumber startingDistance =
      new LoggedTunableNumber("Drive/macroDistance", -1);
  private Transform2d startingPointTransform =
      new Transform2d(startingDistance.get(), 0.0, Rotation2d.fromDegrees(0.0));
  private final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber("Drive/MaxLinearVelocityMPS", 8.0);
  private final LoggedTunableNumber maxLinearAccel =
      new LoggedTunableNumber("Drive/MaxLinearAccelMPSSq", 1.5);
  private final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber("Drive/MaxAngularVelocityRadPS", 8.0);
  private final LoggedTunableNumber maxAngularAccel =
      new LoggedTunableNumber("Drive/MaxAngularAccelRadPSSq", 1.5);
  private PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(
          maxLinearVelocity.get(),
          maxLinearAccel.get(),
          maxAngularVelocity.get(),
          maxAngularAccel.get());

  public Command driveToReefCommand(ReefBranch branch) {
    return defer(
        () -> {
          Pose2d target =
              FieldConstants.Reef.centerFaces[reefSector - 1].transformBy(
                  new Transform2d(
                      FRONT_TO_CENTER_METERS,
                      branch == ReefBranch.LEFT
                          ? -FieldConstants.Reef.centerToBranchAdjustY
                          : FieldConstants.Reef.centerToBranchAdjustY,
                      Rotation2d.k180deg));
          Logger.recordOutput("Drive/ReefTarget", AllianceFlipUtil.apply(target));
          LoggedTunableNumber.ifChanged(
              this,
              (c) -> PATH_CONSTRAINTS = new PathConstraints(c[0], c[1], c[2], c[3]),
              maxLinearVelocity,
              maxLinearAccel,
              maxAngularVelocity,
              maxAngularAccel);
          LoggedTunableNumber.ifChanged(
              this,
              (d) -> {
                startingPointTransform = new Transform2d(d[0], 0.0, Rotation2d.fromDegrees(0));
              },
              startingDistance);
          Pose2d start = target.transformBy(startingPointTransform);
          List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, target);

          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints, PATH_CONSTRAINTS, null, new GoalEndState(0, target.getRotation()));
          return AutoBuilder.pathfindThenFollowPath(path, PATH_CONSTRAINTS);
        });
  }

  public enum ReefBranch {
    LEFT,
    RIGHT,
  }

  public void registerYawConsumer(BiConsumer<Double, Rotation2d> consumer) {
    yawConsumer = consumer;
  }
  // end 5892
}
