package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.PushoutConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Pushout;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.FuelSim;

import org.littletonrobotics.junction.Logger;

import static frc.robot.utils.utils.*;

/**
 * Everything that only exists so the robot can be flown around in AdvantageScope: the
 * FuelSim instance, the fuel the robot is carrying, the shot trigger, and the articulated
 * component poses the custom robot model reads.
 *
 * <p>Constructed and ticked only under {@code RobotBase.isSimulation()}, so none of it
 * runs on the roboRIO.
 */
public class SimRobot {

    /**
     * Every number in here is a guess until someone measures it on the real robot or reads
     * it out of the Onshape model. They change what the simulation looks like, not what the
     * robot does.
     */
    public static final class SimConstants {
        /** Bumper-to-bumper box, used for fuel/robot collision. Read off the exported CAD. */
        public static final double BUMPER_LENGTH_M = 0.817; // robot +X, front to back, 32.2 in
        public static final double BUMPER_WIDTH_M = 0.944; // robot +Y, left to right, 37.2 in
        public static final double BUMPER_HEIGHT_M = Units.inchesToMeters(8.0); // floor to top of bumper

        /** Robot-relative box that swallows fuel while the intake is deployed and running. */
        public static final double INTAKE_REACH_M = Units.inchesToMeters(14.0); // ahead of the front bumper
        public static final double INTAKE_HALF_WIDTH_M = 0.375;

        public static final int FUEL_CAPACITY = 45;

        /** A full field is 408 fuel and every one is a drawn sphere. Thin it for the renderer. */
        public static final int FUEL_ON_FIELD = 200;

        /**
         * Fraction of horizontal speed a rolling fuel loses per second. FuelSim ships 0.1,
         * which is under 10% per second - fuel skated across the field and never settled.
         * Raise for grippier carpet, lower for an ice rink.
         */
        public static final double GROUND_FRICTION_PER_SEC = 1.6;

        /** Centre-to-centre spacing of the starting pile, one fuel diameter plus a little. */
        public static final double FUEL_PILE_SPACING_M = 0.16;

        /**
         * Pushout position past which the intake can collect. NOT PUSHOUT_RETRACTED_POS:
         * AgitateCommand strokes the slide down to 3.0, well under that, so gating there
         * silently stopped the intake working for the whole agitation cycle.
         */
        public static final double INTAKE_DEPLOYED_ROT = PushoutConstants.FULLY_RETRACTED_POS + 1.0;
        public static final double SHOTS_PER_SECOND = 8.0;

        /**
         * Turret rotation axis in robot coordinates. X/Y are the measured TURRET_OFFSET; Z is
         * read off the exported model, whose turret assembly spans 0.36 - 0.59 m above the floor.
         */
        public static final Translation3d TURRET_PIVOT = new Translation3d(
                DrivebaseConstants.TURRET_OFFSET.getX(),
                DrivebaseConstants.TURRET_OFFSET.getY(),
                0.361);

        /** Front edge of the retracted intake in robot coordinates, read off the exported CAD. */
        public static final double INTAKE_FRONT_EDGE_M = 0.332;

        /** Retracted position of the linear slide intake, and how far it travels when deployed. */
        public static final Translation3d INTAKE_HOME = new Translation3d(0.015, 0.0, 0.162);
        public static final double INTAKE_TRAVEL_M = Units.inchesToMeters(12.0);

        /** Where the robot is placed when the simulation starts, and on a reset. */
        public static final Pose2d START_POSE = new Pose2d(2.0, 4.02135, Rotation2d.kZero);

        /** Field extents used to keep the robot inside the guardrails. Matches FuelSim. */
        public static final double FIELD_LENGTH_M = 16.51;
        public static final double FIELD_WIDTH_M = 8.04;

        /**
         * The two BUMP crossings, taken from the same field geometry FuelSim collides fuel
         * against: a pair of ramps either side of each hub, rising to 0.165 m. The hub sits in
         * the gap between the two y bands, which is why this is not a band across the field.
         */
        public static final double BUMP_X_MIN = 3.96;
        public static final double BUMP_X_MAX = 5.18;
        public static final double BUMP_Y_LOW_MIN = 1.57;
        public static final double BUMP_Y_LOW_MAX = 3.42;
        public static final double BUMP_Y_HIGH_MIN = 4.62;
        public static final double BUMP_Y_HIGH_MAX = 6.47;

        /** Fraction of commanded translation speed allowed while crossing a bump. */
        public static final double BUMP_SPEED_SCALE = 0.45;
    }

    /**
     * Solid things on the field the robot can run into, as {minX, maxX, minY, maxY} boxes.
     * Taken from the same geometry FuelSim collides fuel against: the two hub structures and
     * the four trench side blocks. The trench BARS are overhead at 0.565 m and the robot drives
     * under them, so they are deliberately absent.
     */
    public static final double[][] OBSTACLES = {
        // hubs: SIDE 1.2 m square centred on each hub
        {4.61 - 0.6, 4.61 + 0.6, SimConstants.FIELD_WIDTH_M / 2 - 0.6, SimConstants.FIELD_WIDTH_M / 2 + 0.6},
        {SimConstants.FIELD_LENGTH_M - 4.61 - 0.6, SimConstants.FIELD_LENGTH_M - 4.61 + 0.6,
            SimConstants.FIELD_WIDTH_M / 2 - 0.6, SimConstants.FIELD_WIDTH_M / 2 + 0.6},
        // trench side blocks, 0.305 m deep, both ends of both trenches
        {3.96, 5.18, 1.265, 1.265 + 0.305},
        {3.96, 5.18, SimConstants.FIELD_WIDTH_M - 1.57, SimConstants.FIELD_WIDTH_M - 1.57 + 0.305},
        {SimConstants.FIELD_LENGTH_M - 5.18, SimConstants.FIELD_LENGTH_M - 3.96, 1.265, 1.265 + 0.305},
        {SimConstants.FIELD_LENGTH_M - 5.18, SimConstants.FIELD_LENGTH_M - 3.96,
            SimConstants.FIELD_WIDTH_M - 1.57, SimConstants.FIELD_WIDTH_M - 1.57 + 0.305},
    };

    /**
     * How far the intake slide currently sticks out past the FRONT bumper, in metres. Zero when
     * the slide is stowed inside the frame, which it is for most of its travel.
     */
    public static double intakeProtrusionM() {
        if (livePushoutPosition == null) {
            return 0.0;
        }
        double fraction = (livePushoutPosition.getAsDouble() - PushoutConstants.FULLY_RETRACTED_POS)
                / (PushoutConstants.PUSHOUT_EXTENDED_POS - PushoutConstants.FULLY_RETRACTED_POS);
        fraction = Math.max(0.0, Math.min(1.0, fraction));
        double tip = SimConstants.INTAKE_FRONT_EDGE_M + fraction * SimConstants.INTAKE_TRAVEL_M;
        return Math.max(0.0, tip - SimConstants.BUMPER_LENGTH_M / 2.0);
    }

    private static java.util.function.DoubleSupplier livePushoutPosition = null;

    /**
     * True when the robot centre is over either bump crossing. Nothing simulates the ramp
     * itself, so the drivetrain is slowed here instead - crossing a bump flat out is the one
     * thing the sim would otherwise let you do that the real field will not.
     */
    public static boolean isOverBump(double x, double y) {
        boolean inX = (x > SimConstants.BUMP_X_MIN && x < SimConstants.BUMP_X_MAX)
                || (x > SimConstants.FIELD_LENGTH_M - SimConstants.BUMP_X_MAX
                        && x < SimConstants.FIELD_LENGTH_M - SimConstants.BUMP_X_MIN);
        boolean inY = (y > SimConstants.BUMP_Y_LOW_MIN && y < SimConstants.BUMP_Y_LOW_MAX)
                || (y > SimConstants.BUMP_Y_HIGH_MIN && y < SimConstants.BUMP_Y_HIGH_MAX);
        return inX && inY;
    }

    private final SwerveSubsystem drivebase;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Intake intake;
    private final Pushout pushout;
    private final Kicker kicker;

    private final FuelSim fuelSim = new FuelSim("FuelSim");

    private int fuelStored = 0;
    private double shotAccumulator = 0.0;

    public SimRobot(SwerveSubsystem drivebase, Turret turret, Hood hood, Shooter shooter,
            Intake intake, Pushout pushout, Kicker kicker) {
        this.drivebase = drivebase;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.intake = intake;
        this.pushout = pushout;
        this.kicker = kicker;
        livePushoutPosition = pushout::getPosition;

        fuelSim.registerRobot(
                SimConstants.BUMPER_WIDTH_M,
                SimConstants.BUMPER_LENGTH_M,
                SimConstants.BUMPER_HEIGHT_M,
                drivebase::getPose,
                drivebase::getFieldVelocity);

        fuelSim.registerIntake(
                SimConstants.BUMPER_LENGTH_M / 2.0,
                SimConstants.BUMPER_LENGTH_M / 2.0 + SimConstants.INTAKE_REACH_M,
                -SimConstants.INTAKE_HALF_WIDTH_M,
                SimConstants.INTAKE_HALF_WIDTH_M,
                this::canIntake,
                this::onFuelIntaked);

        fuelSim.setGroundFriction(SimConstants.GROUND_FRICTION_PER_SEC);
        fuelSim.useLinearDragWithMagnus(
                ShooterConstants.LINEAR_DRAG_K, ShooterConstants.MAGNUS_LIFT_RATIO);
        fuelSim.setLoggingFrequency(50.0);
        spawnCentrePile();
        fuelSim.start();

        SmartDashboard.putData(Commands.runOnce(this::resetFuel)
                .withName("Reset Fuel").ignoringDisable(true));

        SmartDashboard.putData(Commands.runOnce(this::preloadFuel)
                .withName("Preload Fuel").ignoringDisable(true));

        SmartDashboard.putData(Commands.runOnce(() -> drivebase.resetOdometry(SimConstants.START_POSE))
                .withName("Reset Robot Pose").ignoringDisable(true));

        drivebase.resetOdometry(SimConstants.START_POSE);
    }


    public FuelSim getFuelSim() {
        return fuelSim;
    }

    /** Clears the field, re-spawns the starting fuel, empties the hopper and zeroes both scores. */
    public void resetFuel() {
        fuelSim.clearFuel();
        spawnCentrePile();
        fuelStored = 0;
        FuelSim.Hub.BLUE_HUB.resetScore();
        FuelSim.Hub.RED_HUB.resetScore();
    }

    /** Fills the hopper so shooting can be tested without driving over the pile first. */
    public void preloadFuel() {
        fuelStored = SimConstants.FUEL_CAPACITY;
    }

    /** One pile at midfield, nothing in the depots. */
    private void spawnCentrePile() {
        fuelSim.spawnPile(
                SimConstants.FIELD_LENGTH_M / 2.0,
                SimConstants.FIELD_WIDTH_M / 2.0,
                SimConstants.FUEL_ON_FIELD,
                SimConstants.FUEL_PILE_SPACING_M);
    }

    public int getFuelStored() {
        return fuelStored;
    }

    private boolean canIntake() {
        return fuelStored < SimConstants.FUEL_CAPACITY
                && intake.isIntaking()
                && pushout.getPosition() > SimConstants.INTAKE_DEPLOYED_ROT;
    }

    private void onFuelIntaked() {
        fuelStored++;
    }

    /**
     * Ball speed out of the shooter for a given BOTTOM roller (= motor) RPM, using the
     * two-roller geometry documented in {@link ShooterConstants}.
     */
    public static double ballSpeedMetersPerSecond(double motorRPM) {
        double omega = motorRPM * 2.0 * Math.PI / 60.0;
        double effectiveRadius = (ShooterConstants.ROLLER_RADIUS_BOTTOM_M
                + ShooterConstants.PULLEY_TOP_PER_BOTTOM * ShooterConstants.ROLLER_RADIUS_TOP_M) / 2.0;
        return ShooterConstants.SHOOTER_EFFICIENCY * omega * effectiveRadius;
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void periodic() {
        updateShots();
        fuelSim.updateSim();
        publishComponents();

        Logger.recordOutput("Sim/FuelStored", fuelStored);
        Logger.recordOutput("Sim/FuelOnField", fuelSim.getFuelCount());
        Logger.recordOutput("Sim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("Sim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
        Logger.recordOutput("Sim/BallSpeed", ballSpeedMetersPerSecond(RPSToRPM(shooter.getRPS())));
    }

    private void updateShots() {
        if (!kicker.isFeeding() || fuelStored <= 0) {
            return;
        }

        shotAccumulator += 0.020 * SimConstants.SHOTS_PER_SECOND;
        while (shotAccumulator >= 1.0 && fuelStored > 0) {
            shotAccumulator -= 1.0;
            fuelStored--;
            launchOne();
        }
    }

    private void launchOne() {
        Translation2d turretPos = drivebase.getTurretFieldPosition();
        Translation2d turretVel = drivebase.getTurretFieldVelocity();

        double exitRad = Math.toRadians(90.0 - hood.getAngleDegrees());
        double yawRad = Math.toRadians(turret.getContinuousDegrees())
                + drivebase.getPose().getRotation().getRadians();
        double speed = ballSpeedMetersPerSecond(RPSToRPM(shooter.getRPS()));
        double horizontal = speed * Math.cos(exitRad);

        fuelSim.spawnFuel(
                new Translation3d(turretPos.getX(), turretPos.getY(), DrivebaseConstants.SHOOTER_HEIGHT_M),
                new Translation3d(
                        horizontal * Math.cos(yawRad) + turretVel.getX(),
                        horizontal * Math.sin(yawRad) + turretVel.getY(),
                        speed * Math.sin(exitRad)));
    }

    /**
     * Robot-relative poses for the articulated components of the Robot_Kraken model, in the
     * order they appear in its config.json: turret (hood is baked into that export), then the
     * linear slide intake, which translates along +X rather than pivoting.
     */
    private void publishComponents() {
        Logger.recordOutput("Components/Measured", componentPoses());
        Logger.recordOutput("Components/Zeroed", new Pose3d[] {Pose3d.kZero, Pose3d.kZero});
    }

    private Pose3d[] componentPoses() {
        Pose3d turretPose = new Pose3d(
                SimConstants.TURRET_PIVOT,
                new Rotation3d(0.0, 0.0, Math.toRadians(turret.getContinuousDegrees())));

        double deployFraction = (pushout.getPosition() - PushoutConstants.FULLY_RETRACTED_POS)
                / (PushoutConstants.PUSHOUT_EXTENDED_POS - PushoutConstants.FULLY_RETRACTED_POS);
        deployFraction = Math.max(0.0, Math.min(1.0, deployFraction));
        Pose3d intakePose = new Pose3d(
                SimConstants.INTAKE_HOME.plus(
                        new Translation3d(deployFraction * SimConstants.INTAKE_TRAVEL_M, 0.0, 0.0)),
                Rotation3d.kZero);

        return new Pose3d[] {turretPose, intakePose};
    }
}
