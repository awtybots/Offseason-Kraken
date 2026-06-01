package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AimTurret extends Command {

    private final Turret turret;
    private final SwerveSubsystem swerveSubsystem;

    public AimTurret(Turret turret, SwerveSubsystem swerveSubsystem) {
        this.turret = turret;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(turret); // require turret
    }

    @Override
    public void initialize() {
        swerveSubsystem.isAiming = true;
    }

    private Pose2d getTargetPose() {
        if (swerveSubsystem.isInAllianceZone()) {
            return swerveSubsystem.getDynamicHubLocation(); 
        } else {
            return swerveSubsystem.getDynamicFerryLocation(); 
        }
    }

    @Override
    public void execute() {
        // pick hub or ferry based on robot position
        Pose2d target = getTargetPose();
        Pose2d robotPose = swerveSubsystem.getPose();

        // vector from robot to target
        Translation2d robotToTarget = target.getTranslation().minus(robotPose.getTranslation());

        // angle from robot to target in field space
        Rotation2d fieldAngleToTarget = robotToTarget.getAngle();

        // get the robot relative angle that the turret needs to go to
        double turretTargetDegrees = fieldAngleToTarget.minus(robotPose.getRotation()).getDegrees();

        // convert to setpoint and send to turret
        double setpoint = turret.angleToSetpoint(turretTargetDegrees);
        if (!Double.isNaN(setpoint)) { // only move if theres a valid position to go to
            turret.setAngle(setpoint);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.isAiming = false;
        turret.stopTurret();
    }
}