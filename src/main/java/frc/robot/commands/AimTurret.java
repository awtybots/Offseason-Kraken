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
        Pose2d target = getTargetPose();
        Pose2d robotPose = swerveSubsystem.getPose();

        // use turret's field position instead of robot center
        Translation2d turretPos = swerveSubsystem.getTurretFieldPosition();

        // vector from turret to target
        Translation2d turretToTarget = target.getTranslation().minus(turretPos);

        // angle from turret to target in field space
        Rotation2d fieldAngleToTarget = turretToTarget.getAngle();

        // get the robot relative angle that the turret needs to go to
        double turretTargetDegrees = fieldAngleToTarget.minus(robotPose.getRotation()).getDegrees();

        // convert to setpoint and send to turret
        double setpoint = turret.angleToSetpoint(turretTargetDegrees);
        if (!Double.isNaN(setpoint)) {
            turret.setAngle(setpoint);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
    }
}