package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ControlAllShooting extends Command {

    private final Shooter m_shooter;
    private final Conveyor m_conveyor;
    private final Kicker m_kicker;

    // private final Pushout m_pushout;
    // private final Intake m_intake;
    private final Hood m_hood;
    private final Rollers m_rollers;
    private final Turret m_turret;
    private final SwerveSubsystem drivebase;

    public double distance = 0.0;
    public double recordedTargetRPS = 0.0;
    private boolean isFiring = false;
    private boolean isAtSpeed = false;
    private boolean inShootingZone = true; // false in the opponent alliance zone

    public ControlAllShooting(Shooter shooter, Conveyor conveyor, Kicker kicker, Hood hood,
            Rollers rollers, Turret turret, SwerveSubsystem swerve) {
        this.m_shooter = shooter;
        this.m_conveyor = conveyor;
        this.m_kicker = kicker;
        // this.m_pushout = pushout; commented out because theres no commanding pushout
        // here
        this.m_rollers = rollers;
        // this.m_intake = intake;
        this.m_hood = hood;
        this.m_turret = turret;
        this.drivebase = swerve;

        addRequirements(shooter, conveyor, kicker, rollers); // AimTurret and AimHood do their own things so we
                                                                     // dont need them here
    }

    public boolean isAtSpeed() {
        return isAtSpeed;
    }

    public boolean isFiring() {
        return isFiring;
    }

    private double aimTolerance(double dist) {
        return 1.0;
    }

    private boolean isReadyToFire() { // must be at speed, turret at angle, and hood at angle to shoot
        return inShootingZone
                && isAtSpeed
                && m_hood.isAtAngle()
                && m_turret.isAtAngle()
                && m_turret.isTargetReachable();
    }

    @Override
    public void initialize() {
        isFiring = false;
        isAtSpeed = false;
    }

    @Override
    public void execute() {
        Translation2d turretPos = drivebase.getTurretFieldPosition();
        inShootingZone = !drivebase.isInOpponentAllianceZone();

        if (drivebase.isInAllianceZone()) { // shoot at hub
            Translation2d turretToHub = drivebase.getDynamicHubLocation()
                    .getTranslation().minus(turretPos);
            double dist = turretToHub.getNorm();
            distance = dist;

            double targetRPS = ShooterConstants.hubShooterTable.get(dist);
            recordedTargetRPS = targetRPS;

            m_shooter.setTargetRPS(targetRPS);
            isAtSpeed = Math.abs(m_shooter.getRPS() - targetRPS) <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Hub");
            Logger.recordOutput("Shooting/DistanceToHub", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else if (drivebase.isInNeutralZone()) { // ferry
            Translation2d turretToFerry = drivebase.getDynamicFerryLocation()
                    .getTranslation().minus(turretPos);
            double dist = turretToFerry.getNorm();
            distance = dist;

            double targetRPS = ShooterConstants.ferryShooterTable.get(dist);
            recordedTargetRPS = targetRPS;

            m_shooter.setTargetRPS(targetRPS);
            isAtSpeed = Math.abs(m_shooter.getRPS() - targetRPS) <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Ferry");
            Logger.recordOutput("Shooting/DistanceToFerry", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else { // opponent alliance zone - we never shoot or ferry from here
            recordedTargetRPS = ShooterConstants.ALLIANCE_IDLE_RPS;
            m_shooter.setTargetRPS(ShooterConstants.ALLIANCE_IDLE_RPS);
            isAtSpeed = false;
            Logger.recordOutput("Shooting/Mode", "HoldOpponentZone");
        }
        
        // m_intake.runIntake();

        if (isReadyToFire()) {
            if (!m_turret.isAtCableLimit()) {
                isFiring = true;
                m_kicker.ConveyorToShooter();
                m_conveyor.HopperToShooter();
                m_rollers.RollersToConveyor();
            } else {
                isFiring = false;
                m_kicker.ClearBall();
                m_conveyor.stopConveyor();
                m_rollers.stopRollers();
            }
        } else {
            isFiring = false;
            m_conveyor.stopConveyor();
            m_kicker.stopKicker();
            m_rollers.stopRollers();
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
        Logger.recordOutput("Shooting/InShootingZone", inShootingZone);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released
    }

    @Override
    public void end(boolean interrupted) {

        m_conveyor.stopConveyor();
        m_kicker.stopKicker();
        // m_intake.stopIntake();
        m_rollers.stopRollers();
        isFiring = false;
        isAtSpeed = false;
    }
}