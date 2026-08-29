package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class AimHood extends Command {

    private final Hood hood;
    private final SwerveSubsystem swerveSubsystem;

    public double distance = 0.0;

    public AimHood(Hood hood, SwerveSubsystem swerveSubsystem) {
        this.hood = hood;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
    }

    // under the trench = inside the trench x band AND in one of the two guardrail
    // openings. The x test alone fires anywhere along the hub's x line, including
    // directly in front of the hub.
    private boolean isUnderTrench() {
        Translation2d turretPos = swerveSubsystem.getTurretFieldPosition();
        double x = turretPos.getX();
        double y = turretPos.getY();

        boolean inTrenchX = Math.abs(x - HoodConstants.TRENCH_X_BLUE) <= HoodConstants.TRENCH_THRESHOLD
                || Math.abs(x - HoodConstants.TRENCH_X_RED) <= HoodConstants.TRENCH_THRESHOLD;
        boolean inTrenchY = y <= HoodConstants.TRENCH_Y_RIGHT_MAX
                || y >= HoodConstants.TRENCH_Y_LEFT_MIN;

        return inTrenchX && inTrenchY;
    }

    @Override
    public void execute() {
        Translation2d turretPos = swerveSubsystem.getTurretFieldPosition();

        if (isUnderTrench()) {
            hood.setAngle(HoodConstants.HOOD_MIN_DEGREES);
            Logger.recordOutput("Hood/IsUnderTrench", true);
            Logger.recordOutput("Hood/TurretFieldX", turretPos.getX());
            Logger.recordOutput("Hood/TurretFieldY", turretPos.getY());
            return;
        }

        Logger.recordOutput("Hood/IsUnderTrench", false);
        Logger.recordOutput("Hood/TurretFieldX", turretPos.getX());
        Logger.recordOutput("Hood/TurretFieldY", turretPos.getY());

        if (swerveSubsystem.isInAllianceZone()) {
            Translation2d turretToHub = swerveSubsystem.getDynamicHubLocation()
                    .getTranslation().minus(turretPos);
            double distToHub = turretToHub.getNorm();
            distance = distToHub;

            double targetAngle = HoodConstants.hubHoodTable.get(distToHub);
            hood.setAngle(targetAngle);

            Logger.recordOutput("Hood/Mode", "Hub");
            Logger.recordOutput("Hood/DistanceToHub", distToHub);
            Logger.recordOutput("Hood/TargetAngle", targetAngle);
        } else {
            Translation2d turretToFerry = swerveSubsystem.getDynamicFerryLocation()
                    .getTranslation().minus(turretPos);
            double distToFerry = turretToFerry.getNorm();
            distance = distToFerry;

            double targetAngle = HoodConstants.ferryHoodTable.get(distToFerry);
            hood.setAngle(targetAngle);

            Logger.recordOutput("Hood/Mode", "Ferry");
            Logger.recordOutput("Hood/DistanceToFerry", distToFerry);
            Logger.recordOutput("Hood/TargetAngle", targetAngle);
        }

        Logger.recordOutput("Hood/CurrentAngle", hood.getAngleDegrees());
        Logger.recordOutput("Hood/Distance", distance);
    }

    @Override
    public boolean isFinished() {
        return false; // runs the whole match
    }

    @Override
    public void end(boolean interrupted) {
        hood.goToMin(); // tuck hood down when command ends
    }
}