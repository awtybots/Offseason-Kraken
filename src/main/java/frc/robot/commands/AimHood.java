package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class AimHood extends Command {

    private final Hood hood;
    private final SwerveSubsystem swerveSubsystem;

    public double distance = 0.0;

    private final InterpolatingDoubleTreeMap hubHoodTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ferryHoodTable = new InterpolatingDoubleTreeMap();

    public AimHood(Hood hood, SwerveSubsystem swerveSubsystem) {
        this.hood = hood;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(hood); 

        // aim at hub LUT
        for (var entry : List.of(
                Pair.of(Meters.of(2.0), Degrees.of(20.0)),
                Pair.of(Meters.of(2.5), Degrees.of(23.0)),
                Pair.of(Meters.of(3.0), Degrees.of(26.0)),
                Pair.of(Meters.of(3.5), Degrees.of(30.0)),
                Pair.of(Meters.of(4.0), Degrees.of(34.0)),
                Pair.of(Meters.of(5.0), Degrees.of(40.0)),
                Pair.of(Meters.of(6.0), Degrees.of(45.0))
        )) {
            hubHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        }

        // aim at ferry LUT
        for (var entry : List.of(
                Pair.of(Meters.of(2.0), Degrees.of(20.0)),
                Pair.of(Meters.of(3.0), Degrees.of(25.0)),
                Pair.of(Meters.of(4.0), Degrees.of(30.0)),
                Pair.of(Meters.of(5.0), Degrees.of(35.0)),
                Pair.of(Meters.of(6.0), Degrees.of(40.0))
        )) {
            ferryHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        }
    }

    @Override
    public void initialize() {
    }

    private Translation2d getTurretFieldPosition() { // gets the actual turret's field position 
        Pose2d robotPose = swerveSubsystem.getPose();

        // da robot is 24.5 x 29.5 and da turret is about 3 inches from the back left corner
        double xOffset = (-29.5 / 2.0 + 3.0) * 0.0254; // convert ts to meters
        double yOffset = (24.5 / 2.0 - 3.0) * 0.0254;  

        // get the field relative offset
        Translation2d offset = new Translation2d(xOffset, yOffset).rotateBy(robotPose.getRotation());

        return robotPose.getTranslation().plus(offset);
    }

    private boolean isUnderTrench() { // checks if turret is under the trench by looking at the field x pos
        double x = getTurretFieldPosition().getX();
        return Math.abs(x - HoodConstants.TRENCH_X_BLUE) <= HoodConstants.TRENCH_THRESHOLD
                || Math.abs(x - HoodConstants.TRENCH_X_RED) <= HoodConstants.TRENCH_THRESHOLD;
    }

    @Override
    public void execute() {
        // tuck hood down if turret is near trench so it doesnd decapitate itself
        if (isUnderTrench()) {
            hood.setAngle(HoodConstants.HOOD_MIN_DEGREES);
            Logger.recordOutput("Hood/IsUnderTrench", true);
            Logger.recordOutput("Hood/TurretFieldX", getTurretFieldPosition().getX());
            Logger.recordOutput("Hood/TurretFieldY", getTurretFieldPosition().getY());
            return;
        }

        Logger.recordOutput("Hood/IsUnderTrench", false);
        Logger.recordOutput("Hood/TurretFieldX", getTurretFieldPosition().getX());
        Logger.recordOutput("Hood/TurretFieldY", getTurretFieldPosition().getY());

        // Pose2d robotPose = swerveSubsystem.getPose();

        if (swerveSubsystem.isInAllianceZone()) {
            Translation2d turretToHub = swerveSubsystem.getDynamicHubLocation()
                    .getTranslation().minus(getTurretFieldPosition()); // use turret pos
            double distToHub = turretToHub.getNorm();
            distance = distToHub;

            double targetAngle = hubHoodTable.get(distToHub);
            hood.setAngle(targetAngle);

            Logger.recordOutput("Hood/Mode", "Hub");
            Logger.recordOutput("Hood/DistanceToHub", distToHub);
            Logger.recordOutput("Hood/TargetAngle", targetAngle);
        } 
        else {
            // out of alliance zone — interpolate hood angle from turret position to ferry
            Translation2d turretToFerry = swerveSubsystem.getDynamicFerryLocation()
                    .getTranslation().minus(getTurretFieldPosition()); // use turret pos
            double distToFerry = turretToFerry.getNorm();
            distance = distToFerry;

            double targetAngle = ferryHoodTable.get(distToFerry);
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