package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

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

        boolean reachable = turret.setAngleClamped(turretTargetDegrees);

        Logger.recordOutput("AimTurret/Mode",
                swerveSubsystem.isInAllianceZone() ? "Hub" : "Ferry");
        Logger.recordOutput("AimTurret/TargetPose", target);
        Logger.recordOutput("AimTurret/DistanceM", turretToTarget.getNorm());
        Logger.recordOutput("AimTurret/FieldBearingDeg", fieldAngleToTarget.getDegrees());
        Logger.recordOutput("AimTurret/WantedTurretDeg", turretTargetDegrees);
        Logger.recordOutput("AimTurret/CommandedTurretDeg", turret.getTargetDegrees());
        Logger.recordOutput("AimTurret/ActualTurretDeg", turret.getContinuousDegrees());
        Logger.recordOutput("AimTurret/ErrorDeg",
                turret.getTargetDegrees() - turret.getContinuousDegrees());
        Logger.recordOutput("AimTurret/Reachable", reachable);
        Logger.recordOutput("AimTurret/AtCableLimit", turret.isAtCableLimit());
        Logger.recordOutput("AimTurret/RobotHeadingDeg", robotPose.getRotation().getDegrees());
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