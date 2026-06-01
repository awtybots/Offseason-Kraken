// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BuildConstants; // <---------- WISCONSIN???
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
// import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    private static Robot instance;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Timer disabledTimer;

    public Robot() {
        instance = this;

        // Log WPILib DataLog (SysId, etc.) to the same USB location as AKit logs.
        if (isReal()) {
            DataLogManager.start("/U/logs");
        } else {
            // In sim, log locally to ./logs so we don't depend on a USB mount.
            DataLogManager.start();
        }

        Logger.recordMetadata("ProjectName", "2026Rebuilt");
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDirty", Boolean.toString(BuildConstants.DIRTY != 0));

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.recordMetadata("SimReplayMode", Boolean.toString(Constants.SIM_REPLAY_MODE));
            if (Constants.SIM_REPLAY_MODE) {
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            } else {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
        }

        Logger.start();
    }

    public static Robot getInstance() {
        return instance;
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate the selected container. This will perform all button bindings.
        m_robotContainer = new RobotContainer();

        // Create a timer to disable motor brake a few seconds after disable. This will
        // let the robot stop immediately when disabled, but then also let it be pushed more
        disabledTimer = new Timer();

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Pre-load classes used in deferred RT-trigger bindings so the first in-match
        // press doesn't pay ~58ms of first-use class loading (WPILib units system,
        // InterpolatingDoubleTreeMap, SwerveInputStream.copy, command composition).
        // Construct-only — nothing is scheduled, no motors touched, no side effects.
        try {
            m_robotContainer.warmupCommands();
        } catch (Exception e) {
            Logger.recordOutput("Init/WarmupError", e.getMessage());
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything
        // in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_robotContainer.logControllerInputs();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        m_robotContainer.setUseMegaTag2(true); // Use MT1 during disabled to calibrate heading
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_FRONT, 1); // Seed internal IMU
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_BACK, 1); // Seed internal IMU
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_LEFT, 1); // Seed internal IMU
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_FRONT, 200);
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_BACK, 200);
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_LEFT, 200);
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
            disabledTimer.reset();
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_robotContainer.setUseMegaTag2(true); // Switch to MT2 for accurate x/y with calibrated gyro
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_FRONT, 0);
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_BACK, 0);
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_LEFT, 0);
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_FRONT, 3); // Use internal IMU + external IMU
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_BACK, 3); // Use internal IMU + external IMU
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_LEFT, 3); // Use internal IMU + external IMU
        LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.LIMELIGHT_FRONT, 0.1);
        LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.LIMELIGHT_BACK, 0.1);
        LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.LIMELIGHT_LEFT, 0.1);

        m_robotContainer.setMotorBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Print the selected autonomous command upon autonomous init
        System.out.println("Auto selected: " + m_autonomousCommand);

        // schedule the autonomous command selected in the autoChooser
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_robotContainer.setUseMegaTag2(true); // Switch to MT2 for accurate x/y with calibrated gyro
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_FRONT, 0);
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_FRONT, 3); // Use internal IMU + external IMU
        LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.LIMELIGHT_FRONT, 0.1);
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_BACK, 0);
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_BACK, 3); // Use internal IMU + external IMU
        LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.LIMELIGHT_BACK, 0.1);
        LimelightHelpers.SetThrottle(LimelightConstants.LIMELIGHT_LEFT, 0);
        LimelightHelpers.SetIMUMode(LimelightConstants.LIMELIGHT_LEFT, 3); // Use internal IMU + external IMU
        LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.LIMELIGHT_LEFT, 0.1);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        } else {
            CommandScheduler.getInstance().cancelAll();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {}

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {}
}