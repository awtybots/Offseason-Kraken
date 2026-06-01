// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import java.io.File;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimTurret;
import frc.robot.commands.AimHood;
import frc.robot.commands.ControlAllShooting;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import swervelib.SwerveInputStream;

public class RobotContainer {

    // controllers
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    // subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final Intake m_intake = new Intake();
    private final Hood m_hood = new Hood();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret();
    private final Conveyor m_conveyor = new Conveyor();
    private final Rollers m_rollers = new Rollers();
    private final Pushout m_pushout = new Pushout();
    private final Kicker m_kicker = new Kicker();
    private final HubTrackerSubsystem m_hubtracker = new HubTrackerSubsystem(drivebase, driverXbox);

    // auto choosers
    private SendableChooser<Command> autoChooser;
    private LoggedDashboardChooser<Command> loggedAutoChooser;
    private SendableChooser<Boolean> flipChooser = new SendableChooser<>();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity;

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle;

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented;

    SwerveInputStream driveAngularVelocityKeyboard;
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard;

    private PathConstraints autoConstraints;

    @SuppressWarnings("unused")
    public void warmupCommands() {
        @SuppressWarnings("unused")
        ControlAllShooting shootWarm = new ControlAllShooting(
            m_shooter, m_conveyor, m_kicker, m_pushout, m_intake, m_hood, m_rollers, m_turret, drivebase);
        @SuppressWarnings("unused")
        AimTurret turretWarm = new AimTurret(m_turret, drivebase);
        @SuppressWarnings("unused")
        AimHood hoodWarm = new AimHood(m_hood, drivebase);
    }

    public RobotContainer() {

        configureBindings();

        DriverStation.silenceJoystickConnectionWarning(true);
        SmartDashboard.putNumber("Heading Bias Deg", 0.0);
        SmartDashboard.putBoolean("Is Shooter Running", m_shooter.isShooterRunning());
        SmartDashboard.putNumber("Heading Bias Gain", 0);

        // ==================== NAMED COMMANDS ====================

        NamedCommands.registerCommand("test", Commands.print("I EXIST"));

        // pushout
        NamedCommands.registerCommand("extend", m_pushout.PushCommand());
        NamedCommands.registerCommand("retract intake", m_pushout.RetractCommand());

        // intake
        NamedCommands.registerCommand("intake", m_intake.runIntakeCommand());

        // control all shooting
        NamedCommands.registerCommand("Control All Shooting", Commands.defer(() -> {
            ControlAllShooting shootCmd = new ControlAllShooting(
                    m_shooter, m_conveyor, m_kicker, m_pushout, m_intake, m_hood, m_rollers, m_turret, drivebase);
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        drivebase.setAimLocations();
                        drivebase.isAiming = true;
                    }),
                    Commands.parallel(
                            shootCmd,
                            m_pushout.AgitateWhileIntakingCommand()))
                    .finallyDo(() -> drivebase.isAiming = false);
        }, java.util.Collections.emptySet()).withTimeout(5.3));

        // ==================== AUTO CHOOSER ====================

        flipChooser.setDefaultOption("Not Flipped", false);
        flipChooser.addOption("Flipped", true);
        SmartDashboard.putData("Flip Auto", flipChooser);

        flipChooser.onChange((Boolean flip) -> {
            autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                    autoStream -> autoStream.map(auto -> {
                        auto = new PathPlannerAuto(auto.getName(), flip);
                        return auto;
                    }));
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
            SmartDashboard.putData("Auto Chooser", autoChooser);
            loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
        });

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                autoStream -> autoStream.map(auto -> {
                    auto = new PathPlannerAuto(auto.getName(), flipChooser.getSelected());
                    return auto;
                }));
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
    }

    private void configureBindings() {

        driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> driverXbox.getLeftY() * -1,
                () -> driverXbox.getLeftX() * -1)
                .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true);

        driveDirectAngle = driveAngularVelocity.copy()
                .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
                .headingWhile(true);

        driveRobotOriented = driveAngularVelocity.copy()
                .robotRelative(true)
                .allianceRelativeControl(false);

        driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> -driverXbox.getLeftY(),
                () -> -driverXbox.getLeftX())
                .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                .withControllerHeadingAxis(
                        () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                        () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                .headingWhile(true)
                .translationHeadingOffset(true)
                .translationHeadingOffset(Rotation2d.fromDegrees(0));

        // ==================== DRIVE COMMANDS ====================
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
       
        @SuppressWarnings("unused")
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

        @SuppressWarnings("unused")
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        @SuppressWarnings("unused")
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

        @SuppressWarnings("unused")
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        @SuppressWarnings("unused")
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

        // ==================== DEFAULT COMMANDS ====================

        // turret and hood always aim at hub/ferry the whole match
        m_turret.setDefaultCommand(new AimTurret(m_turret, drivebase));
        m_hood.setDefaultCommand(new AimHood(m_hood, drivebase));

        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else {
            if (Constants.USE_ROBOT_RELATIVE) {
                drivebase.setDefaultCommand(drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
            } else {
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
            }
        }

        // ==================== DRIVER BINDINGS ====================

        // RT shoots
        driverXbox.rightTrigger().whileTrue(
            Commands.defer(() -> {
                ControlAllShooting shootCmd = new ControlAllShooting(
                  m_shooter, m_conveyor, m_kicker, m_pushout, m_intake, m_hood, m_rollers, m_turret, drivebase);
                return Commands.sequence(
                  Commands.runOnce(() -> {
                      drivebase.setAimLocations();
                      drivebase.isAiming = true;
                  }),
                  Commands.parallel(
                    shootCmd,
                    m_pushout.AgitateCommand(),
                    drivebase.lockCommand( // lock wheels while shooting
                      driverXbox::getLeftX,
                      driverXbox::getLeftY,
                      driverXbox::getRightX,
                      driveAngularVelocity::get)
                  ).onlyWhile(() -> !driverXbox.leftTrigger().getAsBoolean() && m_turret.isAtAngle() && m_hood.isAtAngle())
                ).finallyDo(() -> {
                  drivebase.isAiming = false;
                  m_shooter.setTargetRPSCommand(shootCmd.recordedTargetRPS).withTimeout(1.0).schedule(); // keep flywheel spun up briefly after shot
                });
            }, java.util.Collections.emptySet())
        );

        // LT intakes
        driverXbox.leftTrigger().whileTrue(
          Commands.parallel(
            m_pushout.PushCommand(),
            m_intake.runIntakeCommand()
          )
        );

        // LB retracts and stops intake
        driverXbox.leftBumper().whileTrue(
          Commands.parallel(
            m_pushout.RetractCommand(),
            m_intake.stopIntakeCommand()
          )
        );

        // RB unjams
        driverXbox.rightBumper().whileTrue(
          Commands.parallel(
            m_kicker.ReverseKickerCommand(),
            m_conveyor.ReverseConveyorCommand(),
            m_rollers.runReverseRollersCommand()
          )
        );

        // A — outtake
        driverXbox.a().whileTrue(
          Commands.parallel(
            m_intake.runOuttakeCommand(),
            m_rollers.runReverseRollersCommand()
          )
        );

        // POV left — drive to pose
        driverXbox.povLeft().whileTrue(drivebase.driveToPoseDeffered());

        // start zero gyro
        driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

        // ==================== OPERATOR BINDINGS ====================

        Trigger ResetEncoder = operatorXbox.start();

        // reset encoder
        ResetEncoder.onTrue(m_pushout.ResetEncoderCommand());

        // intake
        operatorXbox.x().whileTrue(m_intake.runIntakeCommand());
        operatorXbox.a().whileTrue(
          Commands.parallel(
            m_intake.runOuttakeCommand(),
            m_rollers.runReverseRollersCommand()
          )
        );

        // pushout
        operatorXbox.y().whileTrue(m_pushout.PushoutDutyCycleCommand());
        operatorXbox.b().whileTrue(m_pushout.PushoutDutyCycleRetractCommand());

        // vision
        operatorXbox.povUp().onTrue(drivebase.FrontToggle());
        operatorXbox.povLeft().onTrue(drivebase.LeftToggle());
        operatorXbox.povRight().onTrue(drivebase.VisionToggle());
        operatorXbox.povDown().onTrue(drivebase.BackToggle());

        // ==================== SIMULATION ====================

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
            driveDirectAngleKeyboard.driveToPose(() -> target,
                    new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
                    new ProfiledPIDController(5, 0, 0,
                            new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
            driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox.button(2).whileTrue(Commands.runEnd(
                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
        }

        if (DriverStation.isTest()) {
            if (Constants.USE_ROBOT_RELATIVE) {
                drivebase.setDefaultCommand(drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
            } else {
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
            }
        }
    }

    private double aimTolerance(double distance) {
        if (distance < 2)
            return 5.0;
        else if (distance < 3.5)
            return 2.0;
        return 1.0;
    }

    private ChassisSpeeds applyHeadingBias(ChassisSpeeds speeds) {
        boolean headingBiasEnabled = SmartDashboard.getBoolean("headingBiasEnabled", false);
        if (!headingBiasEnabled)
            return speeds;

        double biasDeg = SmartDashboard.getNumber("Heading Bias Deg", 0.0);
        double gain = SmartDashboard.getNumber("Heading Bias Gain", 0.0);
        double omega = speeds.omegaRadiansPerSecond;

        if (biasDeg != 0.0 && gain != 0.0) {
            double biasRad = Units.degreesToRadians(biasDeg);
            omega += gain * biasRad;
        }

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
    }

    private Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    private boolean isInAllianceZone() {
        Alliance alliance = getAlliance();
        Distance blueZone = Inches.of(182);
        Distance redZone = Inches.of(469);

        if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone))
            return true;
        else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone))
            return true;
        return false;
    }

    private boolean isInOpponentZone() {
        Alliance alliance = getAlliance();
        Distance blueZone = Inches.of(182);
        Distance redZone = Inches.of(469);

        if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().lt(blueZone))
            return true;
        else if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().gt(redZone))
            return true;
        return false;
    }

    private boolean isOnAllianceOutpostSide() {
        Alliance alliance = getAlliance();
        Distance midLine = Inches.of(158.84375);

        if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine))
            return true;
        else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine))
            return true;
        return false;
    }

    private boolean isAimedAt(Pose2d target, double toleranceDegrees) {
        Pose2d robot = drivebase.getPose();
        double targetAngle = Math.toDegrees(Math.atan2(
                target.getY() - robot.getY(),
                target.getX() - robot.getX()));
        double currentAngle = robot.getRotation().getDegrees();
        double error = Math.abs(currentAngle - targetAngle) % 360;
        if (error > 180)
            error = 360 - error;
        return error <= toleranceDegrees;
    }

    public void logControllerInputs() {
        Logger.recordOutput("Input/Driver/LeftX", driverXbox.getLeftX());
        Logger.recordOutput("Input/Driver/LeftY", driverXbox.getLeftY());
        Logger.recordOutput("Input/Driver/RightX", driverXbox.getRightX());
        Logger.recordOutput("Input/Driver/RightY", driverXbox.getRightY());
        Logger.recordOutput("Input/Driver/LeftTrigger", driverXbox.getLeftTriggerAxis());
        Logger.recordOutput("Input/Driver/RightTrigger", driverXbox.getRightTriggerAxis());

        Logger.recordOutput("Input/Operator/LeftX", operatorXbox.getLeftX());
        Logger.recordOutput("Input/Operator/LeftY", operatorXbox.getLeftY());
        Logger.recordOutput("Input/Operator/RightX", operatorXbox.getRightX());
        Logger.recordOutput("Input/Operator/RightY", operatorXbox.getRightY());
        Logger.recordOutput("Input/Operator/LeftTrigger", operatorXbox.getLeftTriggerAxis());
        Logger.recordOutput("Input/Operator/RightTrigger", operatorXbox.getRightTriggerAxis());

        Logger.recordOutput("Shooting/RTHeld", driverXbox.rightTrigger().getAsBoolean());
        Logger.recordOutput("Shooting/InAllianceZone", isInAllianceZone());
    }

    public Command getAutonomousCommand() {
        Command selected = loggedAutoChooser.get();
        if (selected == null)
            return Commands.none();
        return selected;
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public void setUseMegaTag2(boolean use) {
        drivebase.useMegaTag2 = use;
    }
}