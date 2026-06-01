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

    // driver chooser: "David" = port 0 drives, "Asier" = port 1 drives
    private final SendableChooser<String> driverChooser = new SendableChooser<>();

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

    // ========= DRIVER TRIGGERS ===========
    private Trigger RTScore; // shoot / ferry depending on position
    private Trigger RBUnjam; // unjam
    private Trigger LBretract_and_stop; // retract and stop intake
    private Trigger PLDriveToPose; // drive to pose

    private Trigger LT_Intake; // intake

    private Trigger A_runOuttake;


    // ========= OPERATOR TRIGGERS ===========
    private Trigger LT_OP_1900Shot; // fixed speed shot
    private Trigger RT_OP_VariableShoot; // variable shoot

    private Trigger RB_OP_Pass; // pass
    private Trigger LB_OP_unjam; // unjam

    private Trigger X_OP_intake;
    private Trigger A_OP_outtake;

    private Trigger Y_OP_extendIntake;
    private Trigger B_OP_retractIntake;
    private Trigger POVLEFT_OP_agitate;

    private Trigger POVUP_OP_FrontLimelight;
    private Trigger POVLEFT_OP_LeftLimelight;
    private Trigger POVRIGHT_OP_VisionToggle;
    private Trigger POVDown_OP_BackLimelight;

    // -----------------------------------------------------------------------
    // Helpers: resolve which physical controller acts as "driver" vs "operator"
    // based on the SmartDashboard chooser selection.
    // -----------------------------------------------------------------------
    private boolean isAsierSelected() {
        String selected = driverChooser.getSelected();
        return selected != null && selected.equals("Asier");
    }

    /** Returns the controller that should be treated as the driving controller. */
    private CommandXboxController dc() {
        return isAsierSelected() ? operatorXbox : driverXbox;
    }

    /** Returns the controller that should be treated as the operator controller. */
    private CommandXboxController oc() {
        return isAsierSelected() ? driverXbox : operatorXbox;
    }

    public void warmupCommands() {
    @SuppressWarnings("unused")
    ControlAllShooting shootWarm = new ControlAllShooting(
        m_shooter, m_conveyor, m_kicker, m_pushout, m_intake, m_hood, m_rollers, m_turret, drivebase);
    }

    public RobotContainer() {

        // driver chooser
        driverChooser.setDefaultOption("David", "David");
        driverChooser.addOption("Asier", "Asier");
        SmartDashboard.putData("Driver:", driverChooser);

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
                            m_pushout.AgitateWhileIntakingCommand()
                    )
            ).finallyDo(() -> drivebase.isAiming = false);
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
                () -> dc().getLeftY() * -1,
                () -> dc().getLeftX() * -1)
                .withControllerRotationAxis(() -> dc().getRightX() * -1)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true);

        driveDirectAngle = driveAngularVelocity.copy()
                .withControllerHeadingAxis(dc()::getRightX, dc()::getRightY)
                .headingWhile(true);

        driveRobotOriented = driveAngularVelocity.copy()
                .robotRelative(true)
                .allianceRelativeControl(false);

        driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> -dc().getLeftY(),
                () -> -dc().getLeftX())
                .withControllerRotationAxis(() -> dc().getRawAxis(2))
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                .withControllerHeadingAxis(
                        () -> Math.sin(dc().getRawAxis(2) * Math.PI) * (Math.PI * 2),
                        () -> Math.cos(dc().getRawAxis(2) * Math.PI) * (Math.PI * 2))
                .headingWhile(true)
                .translationHeadingOffset(true)
                .translationHeadingOffset(Rotation2d.fromDegrees(0));

        // ========= DRIVER TRIGGERS ===========
        RTScore = dc().rightTrigger(); // shoot / ferry depending on position
        RBUnjam = dc().rightBumper(); // unjam
        LBretract_and_stop = dc().leftBumper(); // retract and stop intake
        PLDriveToPose = dc().povLeft(); // drive to pose

        LT_Intake = dc().leftTrigger(); // intake

        A_runOuttake = dc().a();

        // ========= OPERATOR TRIGGERS ===========
        LT_OP_1900Shot = oc().leftTrigger(); // fixed speed shot
        RT_OP_VariableShoot = oc().rightTrigger(); // variable shoot

        Trigger ResetEncoder = oc().start();

        RB_OP_Pass = oc().rightBumper(); // pass
        LB_OP_unjam = oc().leftBumper(); // unjam

        X_OP_intake = oc().x();
        A_OP_outtake = oc().a();

        Y_OP_extendIntake = oc().y();
        B_OP_retractIntake = oc().b();
        POVLEFT_OP_agitate = oc().povLeft();

        POVUP_OP_FrontLimelight = oc().povUp();
        POVLEFT_OP_LeftLimelight = oc().povLeft();
        POVRIGHT_OP_VisionToggle = oc().povRight();
        POVDown_OP_BackLimelight = oc().povDown();

        // ==================== DRIVE COMMANDS ====================

        Command driveFieldOrientedDirectAngle = drivebase
                .driveFieldOriented(() -> applyHeadingBias(driveDirectAngle.get()));
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(
                () -> applyHeadingBias(driveAngularVelocity.get()));
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(
                () -> applyHeadingBias(driveDirectAngleKeyboard.get()));
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(
                () -> applyHeadingBias(driveAngularVelocityKeyboard.get()));
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
        RTScore.whileTrue(
          Commands.defer(() -> {
            ControlAllShooting shootCmd = new ControlAllShooting(m_shooter, m_conveyor, m_kicker, m_pushout, m_intake, m_hood, m_rollers, m_turret, drivebase);
            return Commands.sequence(
              Commands.runOnce(() -> {
                drivebase.setAimLocations();
                drivebase.isAiming = true;
              }),
              Commands.parallel(
                shootCmd,
                m_pushout.AgitateCommand().onlyWhile(() -> !LT_Intake.getAsBoolean())
                )).finallyDo(() -> Commands.parallel(m_pushout.PushCommand(), Commands.runOnce(() -> drivebase.isAiming = false)));
          }, java.util.Collections.emptySet())
        );

        // LT intakes
        LT_Intake.whileTrue(
          Commands.parallel(
            m_pushout.PushCommand(),
            m_intake.runIntakeCommand()
          )
        );

        // LB retracts and stops intake
        LBretract_and_stop.whileTrue(
          Commands.parallel(
            m_pushout.RetractCommand(),
            m_intake.stopIntakeCommand()
          )
        );

        // RB unjams
        RBUnjam.whileTrue(
          Commands.parallel(
            m_kicker.ReverseKickerCommand(),
            m_conveyor.ReverseConveyorCommand(),
            m_rollers.runReverseRollersCommand()
          )
        );

        // B — agitate manually
        A_runOuttake.whileTrue(
          Commands.parallel(
            m_intake.runOuttakeCommand(),
            m_rollers.runReverseRollersCommand()
          )
        );

        // POV left — drive to pose
        PLDriveToPose.whileTrue(drivebase.driveToPoseDeffered());

        // start zero gyro
        dc().start().onTrue(Commands.runOnce(drivebase::zeroGyro));

        // ==================== OPERATOR BINDINGS ====================

        // reset encoder
        ResetEncoder.onTrue(m_pushout.ResetEncoderCommand());

        // intake
        X_OP_intake.whileTrue(m_intake.runIntakeCommand());
        A_OP_outtake.whileTrue(
          Commands.parallel(
            m_intake.runOuttakeCommand(),
            m_rollers.runReverseRollersCommand()
          )
        );

        // pushout
        Y_OP_extendIntake.whileTrue(m_pushout.PushoutDutyCycleCommand());
        B_OP_retractIntake.whileTrue(m_pushout.PushoutDutyCycleRetractCommand());

        // vision
        POVUP_OP_FrontLimelight.onTrue(drivebase.FrontToggle());
        POVLEFT_OP_LeftLimelight.onTrue(drivebase.LeftToggle());
        POVRIGHT_OP_VisionToggle.onTrue(drivebase.VisionToggle());
        POVDown_OP_BackLimelight.onTrue(drivebase.BackToggle());

        // ==================== SIMULATION ====================

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
            driveDirectAngleKeyboard.driveToPose(() -> target,
                    new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
                    new ProfiledPIDController(5, 0, 0,
                            new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
            dc().start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            dc().button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            dc().button(2).whileTrue(Commands.runEnd(
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
        if (distance < 2) return 5.0;
        else if (distance < 3.5) return 2.0;
        return 1.0;
    }

    private ChassisSpeeds applyHeadingBias(ChassisSpeeds speeds) {
        boolean headingBiasEnabled = SmartDashboard.getBoolean("headingBiasEnabled", false);
        if (!headingBiasEnabled) return speeds;

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

        if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) return true;
        else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) return true;
        return false;
    }

    private boolean isInOpponentZone() {
        Alliance alliance = getAlliance();
        Distance blueZone = Inches.of(182);
        Distance redZone = Inches.of(469);

        if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().lt(blueZone)) return true;
        else if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().gt(redZone)) return true;
        return false;
    }

    private boolean isOnAllianceOutpostSide() {
        Alliance alliance = getAlliance();
        Distance midLine = Inches.of(158.84375);

        if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine)) return true;
        else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine)) return true;
        return false;
    }

    private boolean isAimedAt(Pose2d target, double toleranceDegrees) {
        Pose2d robot = drivebase.getPose();
        double targetAngle = Math.toDegrees(Math.atan2(
                target.getY() - robot.getY(),
                target.getX() - robot.getX()));
        double currentAngle = robot.getRotation().getDegrees();
        double error = Math.abs(currentAngle - targetAngle) % 360;
        if (error > 180) error = 360 - error;
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

        Logger.recordOutput("Shooting/RTHeld", dc().rightTrigger().getAsBoolean());
        Logger.recordOutput("Shooting/InAllianceZone", isInAllianceZone());
    }

    public Command getAutonomousCommand() {
        Command selected = loggedAutoChooser.get();
        if (selected == null) return Commands.none();
        return selected;
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public void setUseMegaTag2(boolean use) {
        drivebase.useMegaTag2 = use;
    }
}