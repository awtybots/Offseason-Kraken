package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import static frc.robot.utils.utils.*;

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
    public double recordedTargetRPM = 0.0;
    private boolean isFiring = false;
    private boolean isAtSpeed = false;
    private boolean inShootingZone = true; // false in the opponent alliance zone
    private double turretAimErrorDegrees = 180.0;

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

    private double aimErrorTo(Translation2d target, Translation2d turretPos) {
        double bearing = target.minus(turretPos).getAngle()
                .minus(drivebase.getPose().getRotation()).getDegrees();
        return Math.abs(MathUtil.inputModulus(
                bearing - m_turret.getContinuousDegrees(), -180.0, 180.0));
    }

    private boolean isReadyToFire() {
        return inShootingZone
                && isAtSpeed
                && m_hood.isAtAngle()
                && turretAimErrorDegrees <= TurretConstants.ANGLE_TOLERANCE_DEGREES
                && m_turret.isTargetReachable();
    }

    @Override
    public void initialize() {
        isFiring = false;
        isAtSpeed = false;
        turretAimErrorDegrees = 180.0;
    }

    @Override
    public void execute() {
        Translation2d turretPos = drivebase.getTurretFieldPosition();
        inShootingZone = !drivebase.isInOpponentAllianceZone();

        if (drivebase.isInAllianceZone()) { // shoot at hub
            Translation2d turretToHub = drivebase.getCachedDynamicHubLocation()
                    .getTranslation().minus(turretPos);
            double dist = turretToHub.getNorm();
            distance = dist;
            turretAimErrorDegrees = aimErrorTo(
                    drivebase.getCachedDynamicHubLocation().getTranslation(), turretPos);

            double targetRPM = ShooterConstants.hubShooterTable.get(dist);
            recordedTargetRPM = targetRPM;

            m_shooter.setTargetRPM(targetRPM);
            isAtSpeed = Math.abs(m_shooter.getRPS() - RPMToRPS(targetRPM)) <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Hub");
            Logger.recordOutput("Shooting/DistanceToHub", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else if (drivebase.isInNeutralZone()) { // ferry
            Translation2d turretToFerry = drivebase.getCachedDynamicFerryLocation()
                    .getTranslation().minus(turretPos);
            double dist = turretToFerry.getNorm();
            distance = dist;
            turretAimErrorDegrees = aimErrorTo(
                    drivebase.getCachedDynamicFerryLocation().getTranslation(), turretPos);

            double targetRPM = ShooterConstants.ferryShooterTable.get(dist);
            recordedTargetRPM = targetRPM;

            m_shooter.setTargetRPM(targetRPM);
            isAtSpeed = Math.abs(m_shooter.getRPS() - RPMToRPS(targetRPM)) <= ShooterConstants.ERROR_MARGIN;

            Logger.recordOutput("Shooting/Mode", "Ferry");
            Logger.recordOutput("Shooting/DistanceToFerry", dist);
            Logger.recordOutput("Shooting/AimTolerance", aimTolerance(dist));
        } else { // opponent alliance zone - we never shoot or ferry from here
            recordedTargetRPM = ShooterConstants.ALLIANCE_IDLE_RPM;
            m_shooter.setTargetRPM(ShooterConstants.ALLIANCE_IDLE_RPM);
            isAtSpeed = false;
            turretAimErrorDegrees = 180.0;
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

        Logger.recordOutput("Shooting/TargetRPM", recordedTargetRPM);
        Logger.recordOutput("Shooting/CurrentRPM", RPSToRPM(m_shooter.getRPS()));
        Logger.recordOutput("Shooting/IsAtSpeed", isAtSpeed);
        Logger.recordOutput("Shooting/IsFiring", isFiring);
        Logger.recordOutput("Shooting/IsReadyToFire", isReadyToFire());
        Logger.recordOutput("Shooting/TurretAtAngle", m_turret.isAtAngle());
        Logger.recordOutput("Shooting/TurretAimErrorDeg", turretAimErrorDegrees);
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