// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
// import java.util.Optional;
// import java.util.ResourceBundle;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
// import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SwerveSubsystem extends SubsystemBase {
  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;

  // Auton Pose tracking for Correction
  private Pose2d targetPathPose = new Pose2d();

  // AdvantageKit: last commanded chassis speeds (used for logging)
  private volatile ChassisSpeeds lastCommandedRobotVelocity = new ChassisSpeeds();
  private volatile ChassisSpeeds lastCommandedFieldVelocity = new ChassisSpeeds();
  // Track yaw over time to estimate yaw rate for logs.


  public int allMegaTagNumber = 1;

  public int frontMegatagNumber = 1;
  public int backMegatagNumber = 1;
  public int leftMegatagNumber = 1;

  public boolean useMegaTag2 = false; // MT1 during disabled, MT2 during auto/teleop
  
  public boolean useMegaTag1 = false; // MT1 during disabled, MT2 during auto/teleop


  public boolean useFrontLimelight = true;
  public boolean useBackLimelight = true;
  public boolean useLeftLimelight = true;

  boolean locked = false;

  // Initialize to non-kZero so YAGSL's SwerveInputStream.aim(supplier) actually registers the target.
  // At registration time it calls supplier.get().equals(Pose2d.kZero) and skips the target if true.
  // Overwritten with real compensated location on first setAimLocations() call.
  private Pose2d cachedDynamicHub = Constants.DrivebaseConstants.getHubPose2D();
  private Pose2d cachedDynamicFerry = Constants.DrivebaseConstants.getFerryPose(new Translation2d());

  

  public boolean visionToggleAll = false;

  // private final SendableChooser<Integer> megaTagChooser = new SendableChooser<Integer>();

  private final SendableChooser<Integer> frontMegatagChooser = new SendableChooser<Integer>();

  private final SendableChooser<Integer> backMegatagChooser = new SendableChooser<Integer>();

  private final SendableChooser<Integer> leftMegatagChooser = new SendableChooser<Integer>();

  private final SendableChooser<Integer> AllMegatagChooser = new SendableChooser<Integer>();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    boolean blueAlliance = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue;
    Pose2d startingPose = blueAlliance
        ? new Pose2d(Meter.of(1), Meter.of(4), Rotation2d.fromDegrees(0))
        : new Pose2d(Meter.of(16), Meter.of(4), Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.stopOdometryThread();

    // Enable heading correction to reduce drift when rotation input is near zero.
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
                                            // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(false,
        false,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
              // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders
            // periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used
    // over the internal encoder and push the offsets onto it. Throws warning if not
    // possible

    swerveDrive.setChassisDiscretization(false, 0.02);
    // The SwerveDrive API in this build does not provide a
    // setChassisVelocityCorrection(...) method,
    // so do not attempt to call it here; rely on the default behavior or other
    // available APIs.
    setupPathPlanner();
    configurePathPlannerLogging();

    // Set up MegaTag Chooser
    frontMegatagChooser.addOption("MegaTag 2", 2);
    frontMegatagChooser.setDefaultOption("MegaTag 1", 1);

    SmartDashboard.putData("Front MegaTag Chooser", frontMegatagChooser);

    backMegatagChooser.addOption("MegaTag 2", 2);
    backMegatagChooser.setDefaultOption("MegaTag 1", 1);

    SmartDashboard.putData("Back MegaTag Chooser", backMegatagChooser);

    leftMegatagChooser.addOption("MegaTag 2", 2);
    leftMegatagChooser.setDefaultOption("MegaTag 1", 1);

    SmartDashboard.putData("Left MegaTag Chooser", leftMegatagChooser);

    AllMegatagChooser.addOption("MegaTag 2", 2);
    AllMegatagChooser.setDefaultOption("MegaTag 1", 1);
    SmartDashboard.putData("MegaTag Chooser", AllMegatagChooser);

    // mark yaaaaaaooo
    // mark yaaaaaaooo
    // mark yaaaaaaooo
    //mark yao and leane programmed all of this
    //ishaan guuuuuuuuuuupta
    // hayden is graduating
    // big dawg is on ALL of the subteams except CAD but that doesnt even ncount
    // on that note why is Anna Yi not CADing????????????????? right now
    // is hayden cooked for IB exams/?
    //yes bro
    //math aa hl --> 6
    //compsci hl--> 6
    //physics hl --> 777777777777777777
  // englisg ll sl --> 6
  //spansih ll a sl --> 5
  // econ sl --> 6
    // Set up bleft Chooser
    // limelightBLeftChooser.addOption("Don't Use B-Left Limelight", false);
    // limelightBLeftChooser.setDefaultOption("Use B-Left Limelight", true);

    // SmartDashboard.putData("B-Left Limelight Chooser", limelightBLeftChooser);
    // // Set up bright Chooser
    // limelightBRightChooser.addOption("Don't Use B-Right Limelight", false);
    // limelightBRightChooser.setDefaultOption("Use B-Right Limelight", true);

    // SmartDashboard.putData("B-Right Limelight Chooser", limelightBRightChooser);
    // // Set up climber Chooser
    // limelightClimberChooser.addOption("Don't Use Climber Limelight", false);
    // limelightClimberChooser.setDefaultOption("Use Climber Limelight", true);

    // SmartDashboard.putData("Climber Limelight Chooser", limelightClimberChooser);
  }
  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg,
        controllerCfg,
        Constants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
            Rotation2d.fromDegrees(0)));
    // Keep behavior consistent in the alternate constructor.

  }

  public Command VisionToggle()
  {
    return run(() ->
    {
      useFrontLimelight = visionToggleAll;
      useBackLimelight = visionToggleAll;
      useLeftLimelight = visionToggleAll;

      visionToggleAll = !visionToggleAll;
    });
  }

  public Command FrontToggle()
  {
    return run(() ->
    {
      useFrontLimelight = !useFrontLimelight;
    });
  }

  public Command BackToggle()
  {
    return run(() ->
    {
      useBackLimelight = !useBackLimelight;
    });
  }

  public Command LeftToggle()
  {
    return run(() ->
    {
      useLeftLimelight = !useLeftLimelight;
    });
  }

  @Override
  public void periodic() {
    // useBLeftLimelight = limelightBLeftChooser.getSelected();
    // useBRightLimelight = limelightBRightChooser.getSelected();
    // useClimberLimelight = limelightClimberChooser.getSelected();


    frontMegatagNumber = frontMegatagChooser.getSelected();
    backMegatagNumber = backMegatagChooser.getSelected();
    leftMegatagNumber = leftMegatagChooser.getSelected();

    if(AllMegatagChooser.getSelected() != allMegaTagNumber)
    {
      allMegaTagNumber = AllMegatagChooser.getSelected();

      frontMegatagNumber = allMegaTagNumber;
      backMegatagNumber = allMegaTagNumber;
      leftMegatagNumber = allMegaTagNumber;
    }
    
    SmartDashboard.putNumber("FrontMegatagNumber", frontMegatagNumber);

    updateOdometry();
    // -----------------------
    // AdvantageKit Logging
    // -----------------------
    Pose2d pose = getPose();

    // Robot pose in field coordinates (x/y meters, yaw degrees).
    Logger.recordOutput("Drive/Pose", pose);
    // Duplicate pose for compatibility with older dashboards.
    Logger.recordOutput("Odometry/Robot", pose);

    // Heading values as structs for AdvantageScope.
    // Estimated heading from pose estimator (preferred for field-relative).
    Logger.recordOutput("Drive/Heading", getHeading());
    // Raw odometry rotation (same as Drive/Heading).
    Logger.recordOutput("Drive/OdometryHeading", pose.getRotation());
    // IMU yaw in degrees; should stay constant when driving straight.
    Logger.recordOutput("Drive/GyroYaw", swerveDrive.getYaw());
    // IMU pitch in degrees; should change on ramps/tilt, not straight drive.
    Logger.recordOutput("Drive/GyroPitch", swerveDrive.getPitch());
    // IMU roll in degrees; should change on side tilt, not straight drive.
    Logger.recordOutput("Drive/GyroRoll", swerveDrive.getRoll());
    // Full IMU rotation (roll/pitch/yaw) as a Rotation3d.
    Logger.recordOutput("Drive/GyroRotation3d", swerveDrive.getGyroRotation3d());

    // Yaw rate estimate (deg/sec); near zero when driving straight.
    // double now = Timer.getFPGATimestamp();
    // double yawRad = swerveDrive.getYaw().getRadians();
    // double dt = now - lastYawTimeSec;
    // double yawRateDegPerSec = 0.0;
    // if (dt > 1e-3) {
    //   yawRateDegPerSec = Units.radiansToDegrees((yawRad - lastYawRadians) / dt);
    //   lastYawRadians = yawRad;
    //   lastYawTimeSec = now;
    // } else if (lastYawTimeSec == 0.0) {
    //   // Initialize on first loop.
    //   lastYawRadians = yawRad;
    //   lastYawTimeSec = now;
    // }
    // Logger.recordOutput("Drive/YawRateDegPerSec", yawRateDegPerSec);

    ChassisSpeeds robotVel = getRobotVelocity();
    ChassisSpeeds fieldVel = getFieldVelocity();
    // Measured chassis velocity in robot frame (vx/vy/omega).
    Logger.recordOutput("Drive/RobotVelocity", robotVel);
    // Measured chassis velocity in field frame (vx/vy/omega).
    Logger.recordOutput("Drive/FieldVelocity", fieldVel);

    // Commanded chassis velocity in robot frame (vx/vy/omega).
    Logger.recordOutput("Drive/CommandedRobotVelocity", lastCommandedRobotVelocity);
    // Commanded chassis velocity in field frame (vx/vy/omega).
    Logger.recordOutput("Drive/CommandedFieldVelocity", lastCommandedFieldVelocity);
    // Commanded rotational rate (rad/sec); should be ~0 when no turn input.
    Logger.recordOutput("Drive/CommandedOmega", lastCommandedRobotVelocity.omegaRadiansPerSecond);
    // Measured rotational rate (rad/sec); should match commanded omega.
    Logger.recordOutput("Drive/ActualOmega", robotVel.omegaRadiansPerSecond);

    // Module states
    Logger.recordOutput("Drive/CurrentModuleStates", swerveDrive.getStates());
    Logger.recordOutput("Drive/DesiredModuleStates",
        swerveDrive.kinematics.toSwerveModuleStates(lastCommandedRobotVelocity));
    // Measured module positions (drive distance + angle).
    Logger.recordOutput("Drive/ModulePositions", swerveDrive.getModulePositions());

      // --- Aim debugging ---
      cachedDynamicHub = getDynamicHubLocation();
      cachedDynamicFerry = getDynamicFerryLocation();
      // The turret is zeroed facing the FRONT of the robot, so turret angle 0 is along the
      // robot's +X and the angle it must hold is just the bearing to the target minus the
      // robot heading - exactly what AimTurret computes. No 180 flip: that used to be here
      // from the drivebase-aim days, when the ROBOT had to point at the hub, and it left
      // these logs reading a half turn away from the error the turret was actually closing.
      // Measured from the turret, not robot centre, so these agree with the aim commands.
      Pose2d dynamicHub = cachedDynamicHub;
      Translation2d turretPos = getTurretFieldPosition();
      Translation2d turretToHub = dynamicHub.getTranslation().minus(turretPos);
      Rotation2d targetAngle = turretToHub.getAngle();
      double aimError = targetAngle.minus(getHeading()).getDegrees();
      double distanceToHub = turretToHub.getNorm();

      Logger.recordOutput("Drive/Aim/DynamicHubPose", dynamicHub);
      Logger.recordOutput("Drive/Aim/StaticHubPose", Constants.DrivebaseConstants.getHubPose2D());
      Logger.recordOutput("Drive/Aim/TargetAngleDeg", targetAngle.getDegrees());
      Logger.recordOutput("Drive/Aim/CurrentHeadingDeg", getHeading().getDegrees());
      Logger.recordOutput("Drive/Aim/TurretAngleForHubDeg", aimError);
      Logger.recordOutput("Drive/Aim/DistanceToHubM", distanceToHub);
      Logger.recordOutput("Drive/Aim/RobotVelX", fieldVel.vxMetersPerSecond);
      Logger.recordOutput("Drive/Aim/RobotVelY", fieldVel.vyMetersPerSecond);
      Translation2d turretVel = getTurretFieldVelocity();
      Logger.recordOutput("Drive/Aim/TurretVelX", turretVel.getX());
      Logger.recordOutput("Drive/Aim/TurretVelY", turretVel.getY());
      Logger.recordOutput("Drive/Aim/OmegaCrossRMagnitude",
          turretVel.minus(new Translation2d(
              fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond)).getNorm());
      Logger.recordOutput("Drive/Aim/IsLocked", locked);

      // --- Ferry aim debugging ---
      Pose2d dynamicFerry = cachedDynamicFerry;
      Translation2d turretToFerry = dynamicFerry.getTranslation().minus(turretPos);
      Rotation2d ferryTargetAngle = turretToFerry.getAngle();
      double ferryAimError = ferryTargetAngle.minus(getHeading()).getDegrees();
      double distanceToFerry = turretToFerry.getNorm();

      Logger.recordOutput("Drive/Aim/DynamicFerryPose", dynamicFerry);
      Logger.recordOutput("Drive/Aim/TurretAngleForFerryDeg", ferryAimError);
      Logger.recordOutput("Drive/Aim/DistanceToFerryM", distanceToFerry);
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            // AdvantageKit: store commanded speeds from PathPlanner (robot-relative)
            lastCommandedRobotVelocity = speedsRobotRelative;
            lastCommandedFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                speedsRobotRelative.vxMetersPerSecond,
                speedsRobotRelative.vyMetersPerSecond,
                speedsRobotRelative.omegaRadiansPerSecond,
                getHeading());
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(10, 0.0, 0.0), //1.5 i swear i see eople saying they shouldnt be lower than 5
              // Translation PID constants
              new PIDConstants(8, 0.0, 0.0) //1
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );
  
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        3.5, 3.5,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  private void configurePathPlannerLogging() {
      PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
          targetPathPose = pose;
      });
  }

  public double getPathFollowingError() {
      return getPose().getTranslation()
                    .getDistance(targetPathPose.getTranslation());
  }

  public boolean  isOffPath(double thresholdMeters) {
      return getPathFollowingError() > thresholdMeters;
  }



  public Command aimAtPose(Pose2d targetPose) {
    return run(() -> {
      // Calculate angle from robot to target
      Translation2d robotPos = getPose().getTranslation();
      Translation2d targetPos = targetPose.getTranslation();
      Rotation2d targetAngle = targetPos.minus(robotPos).getAngle();

      // Drive with zero translation, only rotate toward target
      drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          0, 0,
          swerveDrive.getSwerveController().headingCalculate(
              getHeading().getRadians(), targetAngle.getRadians()),
          getHeading()));
    });
  }
  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
   * PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
   *                                  achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
        swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
            swerveDrive.getStates(),
            DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command
   * ends.
   *
   * @return a Command that tells the robot to drive forward until the command
   *         ends
   */
  public Command driveForward() {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
   * object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother
   *                         controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother
   *                         controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   *                         smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void stop() {
    swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
    
    swerveDrive.setModuleStates((swerveDrive.kinematics.toSwerveModuleStates(
      new ChassisSpeeds(0,0,0))), 
    false);
  }
  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }
  

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      ChassisSpeeds field = velocity.get();
      lastCommandedFieldVelocity = field;
      lastCommandedRobotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
          field.vxMetersPerSecond,
          field.vyMetersPerSecond,
          field.omegaRadiansPerSecond,
          getHeading());
      swerveDrive.driveFieldOriented(field);
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    lastCommandedRobotVelocity = velocity;
    lastCommandedFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
        velocity.vxMetersPerSecond,
        velocity.vyMetersPerSecond,
        velocity.omegaRadiansPerSecond,
        getHeading());
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  private void updateLimelight(String cameraName, int megaTag)
  {
    boolean doRejectUpdate = false;
    if(megaTag == 1) // If using mega tag 1
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      else if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > 0.3)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      else
      {
        // Multi-tag: reject if average distance is too far
        if(mt1.avgTagDist > 3)
        {
          doRejectUpdate = true;
        }
      }

      // Reject if spinning fast
      if(Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 360)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        // Scale std devs by distance: close tags = more trust, far tags = less trust
        double dist = mt1.avgTagDist;
        double xyStd = 0.3 + (dist * dist * 0.15);
        // Multi-tag is more reliable, so reduce std devs
        if(mt1.tagCount >= 2) xyStd *= 0.5;
        // Trust vision more while disabled to lock in pose before match
        if(DriverStation.isDisabled()) xyStd *= 0.25;

        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStd, xyStd, 9999999));
        swerveDrive.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else  // If using mega tag 2
    {
      LimelightHelpers.SetRobotOrientation(cameraName,
      swerveDrive.getOdometryHeading().getDegrees(),
      0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

      if(Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 720)
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      // Reject if average tag distance is too far for reliable MT2
      if(mt2.avgTagDist > 3)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        // Scale std devs by distance and tag count
        double dist = mt2.avgTagDist;
        double xyStd = 0.3 + (dist * dist * 0.1);
        if(mt2.tagCount >= 2) xyStd *= 0.5;
        // Trust vision more while disabled to lock in pose before match
        if(DriverStation.isDisabled()) xyStd *= 0.25;

        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStd, xyStd, 9999999));
        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {


    if(useBackLimelight) updateLimelight(LimelightConstants.LIMELIGHT_BACK, backMegatagNumber);
    if(useFrontLimelight) updateLimelight(LimelightConstants.LIMELIGHT_FRONT, frontMegatagNumber);
    if(useLeftLimelight) updateLimelight(LimelightConstants.LIMELIGHT_LEFT, leftMegatagNumber);
    
    swerveDrive.updateOdometry();
  }

  /**
   * 
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is
   *         available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // public Supplier<Boolean> canShoot() {
  //   // Zone bounds in meters - adjust these to the desired shooting area
  //   final double minX = 3.0;
  //   final double maxX = 3.5;
  //   final double minY = 3.5;
  //   final double maxY = 4.5;

  //   Pose2d pose = getPose();
  //   boolean inZone = pose.getX() >= minX && pose.getX() <= maxX
  //       && pose.getY() >= minY && pose.getY() <= maxY;


    // if (inZone) {
    //   return () -> false;
    // } else {
    //   return () -> true;
    //   }
    // }
  

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    SmartDashboard.putBoolean("Wheel Lock", true);
    swerveDrive.lockPose();
  }

  public Command lockCommand () {
    
    // Commands.waitSeconds(2);
     return Commands.runOnce(this::lock, this).repeatedly();
  }

  public Command lockCommand(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, Supplier<ChassisSpeeds> fieldOrientedSpeeds) {
    return run(() -> {
        double leftMag = Math.hypot(leftX.getAsDouble(), leftY.getAsDouble());
        double rightMag = Math.abs(rightX.getAsDouble());
        if ((leftMag + rightMag) > Constants.OperatorConstants.DEADBAND) {
            driveFieldOriented(fieldOrientedSpeeds.get());
            locked = false;
        } 
        else {
          lock();
          locked = true;
        }
    }).finallyDo(interrupted -> stop());
  }


  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Pose2d getCachedDynamicHubLocation() {
    return cachedDynamicHub;
  }

  public Pose2d getCachedDynamicFerryLocation() {
    return cachedDynamicFerry;
  }

  /**
   * Computes a virtual hub location that compensates for robot velocity,
   * so the robot aims ahead of the actual hub when moving (shoot-on-the-move).
   * Uses an iterative time-of-flight lookup to converge on the correct lead.
   *
   * @return A Pose2d representing the compensated aim point.
   */
  /**
 * Computes a virtual hub location that compensates for robot velocity
 * (shoot-on-the-move) and robot tilt (shoot-on-the-tilt).
 * Uses an iterative time-of-flight lookup to converge on the correct lead.
 *
 * @return A Pose2d representing the compensated aim point.
 */
    public Pose2d getDynamicHubLocation() {

        Translation2d hubVec = Constants.DrivebaseConstants.getHubPose2D().getTranslation();
        Translation2d robotVec = getTurretFieldPosition();
        Translation2d robotVel = getTurretFieldVelocity(); // includes omega x r

        // velocity compensation — iterative TOF convergence
        Translation2d CompensatedHub = hubVec;
        for (int i = 0; i < 15; i++) {
            double distance = CompensatedHub.minus(robotVec).getNorm();
            double tof = Constants.ShooterConstants.TOF.get(distance);
            CompensatedHub = hubVec.minus(robotVel.times(tof));
        }

        // // tilt compensation
        // double pitchRad = swerveDrive.getPitch().getRadians(); // tilt forward/back
        // double rollRad = swerveDrive.getRoll().getRadians();   // tilt left/right

        // if (Math.abs(pitchRad) > Math.toRadians(1.0) || Math.abs(rollRad) > Math.toRadians(1.0)) {

        //     // hub target height is fixed, shooter height changes with tilt
        //     // based on shooter position: back left corner, 3" in from each edge
        //     // robot 29.5" long x 24.5" wide → offsets from center:
        //     // fwd = -(29.5/2 - 3) = -11.75", left = (24.5/2 - 3) = 9.25"
        //     double actualShooterHeight = Constants.DrivebaseConstants.SHOOTER_HEIGHT_M
        //             + Constants.DrivebaseConstants.SHOOTER_OFFSET_FWD_M * Math.sin(pitchRad)
        //             + Constants.DrivebaseConstants.SHOOTER_OFFSET_LEFT_M * Math.sin(rollRad);

        //     // height difference between shooter exit and hub target
        //     double dz = Constants.DrivebaseConstants.getHubPose3D().getZ() - actualShooterHeight;

        //     // vector from robot to compensated hub in field space
        //     Translation2d robotToHub = CompensatedHub.minus(robotVec);
        //     double dx = robotToHub.getX();
        //     double dy = robotToHub.getY();

        //     // robot heading so we can decompose into forward/side components
        //     double yaw = getHeading().getRadians();

        //     // project dx/dy into robot-relative forward and left components
        //     double fwd  =  dx * Math.cos(yaw) + dy * Math.sin(yaw); // forward in robot frame
        //     double left = -dx * Math.sin(yaw) + dy * Math.cos(yaw); // left in robot frame

        //     // pitch forward → shooter tilts down → hub appears further → correct fwd
        //     // roll left → shooter tilts left → hub appears shifted → correct left
        //     double correctedFwd  = fwd  + dz * Math.tan(pitchRad);
        //     double correctedLeft = left - dz * Math.tan(rollRad);

        //     // convert back to field coordinates
        //     double correctedDx = correctedFwd * Math.cos(yaw) - correctedLeft * Math.sin(yaw);
        //     double correctedDy = correctedFwd * Math.sin(yaw) + correctedLeft * Math.cos(yaw);

        //     CompensatedHub = robotVec.plus(new Translation2d(correctedDx, correctedDy));

        //     Logger.recordOutput("Drive/Tilt/PitchDeg", Math.toDegrees(pitchRad));
        //     Logger.recordOutput("Drive/Tilt/RollDeg", Math.toDegrees(rollRad));
        //     Logger.recordOutput("Drive/Tilt/ActualShooterHeightM", actualShooterHeight);
        //     Logger.recordOutput("Drive/Tilt/DzM", dz);
        //     Logger.recordOutput("Drive/Tilt/CorrectedFwd", correctedFwd);
        //     Logger.recordOutput("Drive/Tilt/CorrectedLeft", correctedLeft);
        // }

        Rotation2d aimRotation = CompensatedHub.minus(robotVec).getAngle();
        return new Pose2d(CompensatedHub, aimRotation);
    }

    /**
     * Field-frame velocity of the TURRET, which is what the ball actually inherits at
     * release - not the velocity of the robot's centre of rotation.
     *
     * <p>{@code v_turret = v_chassis + omega x r}, with {@code r} the turret offset rotated
     * into the field frame. In 2D, {@code omega x r = omega * (-r.y, r.x)}.
     *
     * <p>At |r| = 0.237 m a spin of 8 rad/s adds 1.9 m/s - comparable to translation speed,
     * and it does not vanish when the robot is driving straight, only when it is not turning.
     */
    public Translation2d getTurretFieldVelocity() {
      ChassisSpeeds fieldVel = getFieldVelocity();
      Translation2d r = Constants.DrivebaseConstants.TURRET_OFFSET
          .rotateBy(getPose().getRotation());
      double omega = fieldVel.omegaRadiansPerSecond;
      return new Translation2d(
          fieldVel.vxMetersPerSecond - omega * r.getY(),
          fieldVel.vyMetersPerSecond + omega * r.getX());
    }

    public Translation2d getTurretFieldPosition() {
      Pose2d robotPose = getPose();
      // double xOffset = (-29.5 / 2.0 + 3.0) * 0.0254;
      // double yOffset = (24.5 / 2.0 - 3.0) * 0.0254;
      // Translation2d offset = new Translation2d(xOffset, yOffset).rotateBy(robotPose.getRotation());
       return robotPose.getTranslation().plus(
            Constants.DrivebaseConstants.TURRET_OFFSET.rotateBy(robotPose.getRotation()));
    }

  // test 2

  /**
   * Computes a virtual ferry location that compensates for robot velocity,
   * so the robot aims ahead of the actual ferry target when moving (pass-on-the-move).
   * Uses an iterative time-of-flight lookup to converge on the correct lead.
   *
   * @return A Pose2d representing the compensated aim point.
   */
  public Pose2d getDynamicFerryLocation() {
    Translation2d ferryVec = Constants.DrivebaseConstants.getFerryPose(getPose().getTranslation()).getTranslation();
    Translation2d robotVec = getTurretFieldPosition();
    Translation2d robotVel = getTurretFieldVelocity(); // includes omega x r

    Translation2d CompensatedFerry = ferryVec;
    for (int i = 0; i < 4; i++) {
      double distance = CompensatedFerry.minus(robotVec).getNorm();
      // ferryTOF, not the hub TOF map: the hub map only spans 2-6 m and clamps, so
      // every pass past 6 m used to lead with the 6 m hub flight time.
      double tof = Constants.ShooterConstants.ferryTOF.get(distance);
      CompensatedFerry = ferryVec.minus(robotVel.times(tof));
    }

    return new Pose2d(CompensatedFerry, new Rotation2d());
  }

  private boolean IsOnLeftSide()
  {
      return getPose().getY() > 4;
  }


  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  public boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    double x = getPose().getX();

    if (alliance == Alliance.Blue && x < Constants.DrivebaseConstants.BLUE_ALLIANCE_ZONE_X_M) {
      return true;
    } else if (alliance == Alliance.Red && x > Constants.DrivebaseConstants.RED_ALLIANCE_ZONE_X_M) {
      return true;
    }

    return false;
  }

  /**
   * True when the robot is between the two alliance zones. We ferry only from here -
   * never from the opponent's alliance zone - so this bounds ferry distance to about
   * 1.8-10.6 m, which is the domain ferryTOF and the ferry tables are built over.
   */
  public boolean isInNeutralZone() {
    double x = getPose().getX();
    return x > Constants.DrivebaseConstants.BLUE_ALLIANCE_ZONE_X_M
        && x < Constants.DrivebaseConstants.RED_ALLIANCE_ZONE_X_M;
  }

  /** True when the robot is deep in the opponent's alliance zone, where we never shoot. */
  public boolean isInOpponentAllianceZone() {
    return !isInAllianceZone() && !isInNeutralZone();
  }

  private Pose2d GetDriveToPose()
  {
      boolean isInAllianceZone = isInAllianceZone();
      boolean IsOnLeftSide = IsOnLeftSide();

      if(isInAllianceZone)
      {
          if(IsOnLeftSide)
          {
            return new Pose2d(new Translation2d(3.478, 7.432),
              Rotation2d.fromDegrees(0));
          }
          else
          {
            return new Pose2d(new Translation2d(3.478, 0.67),
              Rotation2d.fromDegrees(0));
          }
      }

      else
      {
        if(IsOnLeftSide)
        {
          return new Pose2d(new Translation2d(5.789, 7.432),
              Rotation2d.fromDegrees(180));
        }
        else
        {
          return new Pose2d(new Translation2d(5.789, 0.67),
              Rotation2d.fromDegrees(180));
        }
      }
  }

  public Command driveToPoseDeffered()
  {
    return defer(() -> driveToPose(GetDriveToPose()));
  }

  public Field2d getField()
  {
    return swerveDrive.field;
  }
}
