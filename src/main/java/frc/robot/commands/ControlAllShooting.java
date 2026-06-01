package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class ControlAllShooting extends Command {

    private final Shooter m_shooter;
    private final Conveyor m_conveyor;
    private final Kicker m_kicker;
    private final Pushout m_pushout;
    private final Intake m_intake;
    private final Hood m_hood;
    private final Rollers m_rollers;
    private final Turret m_turret;
    private final SwerveSubsystem m_swervesubsystem;

    public double distance = 0.0;
    public double recordedTargetRPS = 0.0;
    private boolean isFiring = false;
    private boolean isAtSpeed = false;

    // maps distance to shooter RPS
    private final InterpolatingDoubleTreeMap hubShooterTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ferryShooterTable = new InterpolatingDoubleTreeMap();

    public ControlAllShooting(Shooter shooter, Conveyor conveyor, Kicker kicker, Pushout pushout, Intake intake, Hood hood, Rollers rollers, Turret turret, SwerveSubsystem swerve)
    {
        this.m_shooter = shooter;
        this.m_conveyor = conveyor;
        this.m_kicker = kicker;
        this.m_pushout = pushout;
        this.m_rollers = rollers;
        this.m_intake = intake;
        this.m_hood = hood;
        this.m_turret = turret;
        this.m_swervesubsystem = swerve;

        addRequirements(shooter, conveyor, kicker, pushout, intake, rollers); // AimTurret and AimHood do their own things so we dont need them here

        for (var entry : List.of(
                Pair.of(Meters.of(2.0), RPM.of(1700)),
                Pair.of(Meters.of(2.5), RPM.of(1935)),
                Pair.of(Meters.of(3.0), RPM.of(2010)),
                Pair.of(Meters.of(3.5), RPM.of(2150)),
                Pair.of(Meters.of(4.0), RPM.of(2280)),
                Pair.of(Meters.of(5.2), RPM.of(2517)),
                Pair.of(Meters.of(6.0), RPM.of(3060))
        )) {
            hubShooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM) / 60.0); // convert to RPS
        }

        // ferry table is gonna be diff angles and rpms so tune
        for (var entry : List.of(
                Pair.of(Meters.of(2.0), RPM.of(1500)),
                Pair.of(Meters.of(3.0), RPM.of(1800)),
                Pair.of(Meters.of(4.0), RPM.of(2100)),
                Pair.of(Meters.of(5.0), RPM.of(2400)),
                Pair.of(Meters.of(6.0), RPM.of(2700))
        )) {
            ferryShooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM) / 60.0); // convert to RPS
        }
    }

    public boolean isAtSpeed() {
        return isAtSpeed;
    }

    public boolean isFiring() {
        return isFiring;
    }

    private double aimTolerance(double dist) {
        if (dist < 2.5) return 2.5;
        else if (dist < 3.5) return 2;
        else if (dist < 5.0) return 1.5;
        else return 1;
    }

    private boolean isReadyToFire() { // must be at speed, turret at angle, and hood at angle to shoot
        return isAtSpeed
        && m_hood.isAtAngle()
        && m_turret.isAtAngle();
    }

    @Override
    public void initialize() {
        isFiring = false;
        isAtSpeed = false;
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_swervesubsystem.getPose();

        if (m_swervesubsystem.isInAllianceZone()) { // shoot at hub
            Translation2d turretToHub = m_swervesubsystem.getDynamicHubLocation()
                    .getTranslation().minus(robotPose.getTranslation());
            double dist = turretToHub.getNorm();
            distance = dist;

            double targetRPS = hubShooterTable.get(dist); // look up RPS from distance
            recordedTargetRPS = targetRPS;

            m_shooter.setTargetRPS(targetRPS);
            isAtSpeed = Math.abs(m_shooter.getRPS() - targetRPS) <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Hub");
            Logger.recordOutput("Shooting/DistanceToHub", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else { // ferry
            Translation2d turretToFerry = m_swervesubsystem.getDynamicFerryLocation()
                    .getTranslation().minus(robotPose.getTranslation());
            double dist = turretToFerry.getNorm();
            distance = dist;

            double targetRPS = ferryShooterTable.get(dist); // look up RPS from distance
            recordedTargetRPS = targetRPS;

            m_shooter.setTargetRPS(targetRPS);
            isAtSpeed = Math.abs(m_shooter.getRPS() - targetRPS) <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Ferry");
            Logger.recordOutput("Shooting/DistanceToFerry", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        }

        if (isReadyToFire()) {
            if (!m_turret.isAtCableLimit()) { // only feed balls if turret is not wrapping
                // get da balls into da shooter
                isFiring = true;
                m_kicker.ConveyorToShooter();
                m_conveyor.HopperToShooter();
                m_rollers.RollersToConveyor();
                m_intake.runIntake();
                m_pushout.PushIntake();
            } else {
                // turret is wrapping — stop feeding but shooter and hood keep running
                isFiring = false;
                m_kicker.ClearBallCommand();
                m_conveyor.stopConveyor();
                m_intake.stopIntake();
                m_rollers.stopRollers();
                m_pushout.PushIntake();
            }
        } else {
            isFiring = false;
            m_conveyor.stopConveyor();
            m_kicker.stopKicker();
            m_intake.stopIntake();
            m_rollers.stopRollers();
            m_pushout.PushIntake();
        }

        Logger.recordOutput("Shooting/TargetRPS", recordedTargetRPS);
        Logger.recordOutput("Shooting/CurrentRPS", m_shooter.getRPS());
        Logger.recordOutput("Shooting/IsAtSpeed", isAtSpeed);
        Logger.recordOutput("Shooting/IsFiring", isFiring);
        Logger.recordOutput("Shooting/IsReadyToFire", isReadyToFire());
        Logger.recordOutput("Shooting/TurretAtAngle", m_turret.isAtAngle());
        Logger.recordOutput("Shooting/HoodAtAngle", m_hood.isAtAngle());
        Logger.recordOutput("Shooting/TurretAtCableLimit", m_turret.isAtCableLimit());
        Logger.recordOutput("Shooting/Distance", distance);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooting();
        m_conveyor.stopConveyor();
        m_kicker.stopKicker();
        m_intake.stopIntake();
        m_rollers.stopRollers();
        isFiring = false;
        isAtSpeed = false;
    }
}