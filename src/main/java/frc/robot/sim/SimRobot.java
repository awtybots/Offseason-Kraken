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
        // The intake slide is a smooth sheet, not bumper fabric, so it should skate along a
        // structure rather than catch on it the way the bumper does. maple-sim's bumper runs at
        // 0.65 (AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION).
        public static final double INTAKE_FRICTION = 0.40;
        public static final double INTAKE_HALF_WIDTH_M = 0.375;

        public static final int FUEL_CAPACITY = 45;

        // Hopper interior in ROBOT frame, taken from the model's own hopper walls. The glb is
        // Y-up and AdvantageScope rotates it x:90 then z:180, so robotX = -glbX, robotY = glbZ,
        // robotZ = glbY + 0.101. Side walls (right hopper / Part 9) land at Y = +/-0.38, the back
        // wall at X = -0.09, the lid (small top hopper) at Z = 0.571, interior floor at Z = 0.054.
        public static final double HOPPER_X_MIN = -0.09;
        public static final double HOPPER_X_MAX = 0.31;
        public static final double HOPPER_Y_HALF = 0.38;
        public static final double HOPPER_Z_MIN = 0.054;
        public static final double HOPPER_Z_MAX = 0.571;
        public static final double FUEL_RADIUS_M = 0.075;

        // Teleop feel in simulation only; the real robot keeps full stick authority. Translation
        // was outrunning the field, and with it backed off the rotation axis gets a small boost
        // so turning still feels quick relative to driving.
        public static final double SIM_TRANSLATION_SCALE = 0.70;
        public static final double SIM_ROTATION_SCALE = 1.10;

        /**
         * Only used by the single-pile helper. The default layout is FuelSim's full 408-fuel
         * field stock; every fuel is a drawn sphere, so lower this if rendering struggles.
         */
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
     * Every number here was measured off AdvantageScope's own 2026 field model by height band -
     * compose the node transforms, then keep only what reaches below the robot's ~0.6 m.
     */
    public static final double[][] OBSTACLES = {
        // The hub is a 1.2 m square, solid from the floor to 1.27 m: "Hub Side Panel" spans
        // x 4.02-5.20 and "Hub Rear Panel" sits at x 5.20-5.21. FuelSim agrees independently -
        // Hub.SIDE = 1.2 centred on (4.61, 4.02). An earlier box ran to x 5.49, which put 0.28 m
        // of invisible wall behind each hub.
        {4.01, 5.21, 3.42, 4.62},
        {SimConstants.FIELD_LENGTH_M - 5.21, SimConstants.FIELD_LENGTH_M - 4.01, 3.42, 4.62},
        // The climbing tower, which stands at the alliance wall and NOT beside the hub. The only
        // part of it at robot height is the pair of uprights on the line x 1.00-1.09, at
        // y 3.28-3.32 and y 4.14-4.18; the 0.82 m gap between them is narrower than this 0.944 m
        // robot, so the pair behaves as one wall. Its rung is at 0.66 m and its base plate is
        // 0.06 m, so the robot passes under one and over the other. Widened ~5 cm a side because
        // a box thinner than one loop of travel can be tunnelled through.
        //
        // NOTE the field model is Y-up with +X toward red, so model +Z is field MINUS Y:
        // fieldY = 4.02 - modelZ. Getting that sign backwards put this box 0.58 m to the +y side
        // - it was the RED tower's y span on the blue tower. The hub could never catch it,
        // because the hub is symmetric about the field centreline and the tower is not.
        {0.95, 1.14, 3.28, 4.18},
        {SimConstants.FIELD_LENGTH_M - 1.14, SimConstants.FIELD_LENGTH_M - 0.95,
            SimConstants.FIELD_WIDTH_M - 4.18, SimConstants.FIELD_WIDTH_M - 3.28},
        // Trench side blocks, 0.305 m deep, both ends of both trenches. These match FuelSim's
        // own trench geometry exactly. The trench BARS are overhead at 0.565 m and the robot
        // drives under them, so they are deliberately absent.
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
    private int hubShotsFired = 0;

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
        // 408 fuel logged as Translation3d is ~9.6 KB a sample, so 50 Hz was pushing half a
        // megabyte a second into a history AdvantageScope keeps in memory for scrubbing -
        // that, not the reset itself, is what makes a long session go sluggish. Upstream
        // defaults to 10 Hz; 25 halves the rate and still updates faster than the eye.
        fuelSim.setLoggingFrequency(25.0);
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
        hubShotsFired = 0;
        FuelSim.Hub.BLUE_HUB.resetScore();
        FuelSim.Hub.RED_HUB.resetScore();
    }

    /** Our own hub, so the tally follows the alliance rather than always reading blue. */
    private static FuelSim.Hub ourHub() {
        return edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue)
                == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                ? FuelSim.Hub.RED_HUB : FuelSim.Hub.BLUE_HUB;
    }

    /** Fills the hopper so shooting can be tested without driving over the pile first. */
    public void preloadFuel() {
        fuelStored = SimConstants.FUEL_CAPACITY;
    }

    /**
     * The normal field layout: FuelSim's own starting stock, 408 fuel across the neutral-zone
     * pile and both depots. {@code spawnPile} is still there if a single pile is wanted.
     */
    private void spawnCentrePile() {
        fuelSim.spawnStartingFuel();
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
        Logger.recordOutput("Sim/HeldFuel", heldFuelPositions());
        Logger.recordOutput("Sim/FuelOnField", fuelSim.getFuelCount());
        Logger.recordOutput("Sim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("Sim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
        Logger.recordOutput("Sim/BallSpeed", ballSpeedMetersPerSecond(RPSToRPM(shooter.getRPS())));
        Logger.recordOutput("Sim/AimTrajectory", aimTrajectory());

        // Hub accuracy while aiming at the hub from inside our own alliance zone. Misses lag by
        // roughly one flight time, because a shot counts as fired the instant it leaves.
        int scored = ourHub().getScore();
        Logger.recordOutput("Sim/HubShotsFired", hubShotsFired);
        Logger.recordOutput("Sim/HubShotsScored", scored);
        Logger.recordOutput("Sim/HubShotsMissed", Math.max(0, hubShotsFired - scored));
        Logger.recordOutput("Sim/HubAccuracyPct",
                hubShotsFired == 0 ? 0.0 : 100.0 * scored / hubShotsFired);
    }

    /**
     * Where the carried fuel sits inside the hopper, in field coordinates.
     *
     * <p>Published as {@code Translation3d[]}, the same shape FuelSim uses for loose fuel, so the
     * held balls can be dropped into the 3D field with the same game-piece setup and the hopper
     * fills up as the robot intakes.
     *
     * <p>Columns are sized from the real hopper volume at one ball diameter, but the LAYER
     * spacing compresses once the count outgrows the box. The alternative is balls stacked up
     * through the lid, and a hopper that reads as visibly packed is the useful signal here.
     */
    public Translation3d[] heldFuelPositions() {
        if (fuelStored <= 0) {
            return new Translation3d[0];
        }
        double r = SimConstants.FUEL_RADIUS_M;
        double xLo = SimConstants.HOPPER_X_MIN + r;
        double xHi = SimConstants.HOPPER_X_MAX - r;
        double yLo = -SimConstants.HOPPER_Y_HALF + r;
        double yHi = SimConstants.HOPPER_Y_HALF - r;
        double zLo = SimConstants.HOPPER_Z_MIN + r;
        double zHi = SimConstants.HOPPER_Z_MAX - r;

        int nx = Math.max(1, (int) Math.floor((xHi - xLo) / (2.0 * r)) + 1);
        int ny = Math.max(1, (int) Math.floor((yHi - yLo) / (2.0 * r)) + 1);
        int perLayer = nx * ny;
        int layers = Math.max(1, (int) Math.ceil((double) fuelStored / perLayer));

        Pose2d pose = drivebase.getPose();
        Translation3d[] out = new Translation3d[fuelStored];
        for (int i = 0; i < fuelStored; i++) {
            int layer = i / perLayer;
            int slot = i % perLayer;
            int ix = slot % nx;
            int iy = slot / nx;
            double x = nx == 1 ? (xLo + xHi) / 2.0 : xLo + ix * (xHi - xLo) / (nx - 1);
            double y = ny == 1 ? (yLo + yHi) / 2.0 : yLo + iy * (yHi - yLo) / (ny - 1);
            double z = layers == 1 ? zLo : zLo + layer * (zHi - zLo) / (layers - 1);
            Translation2d inField = new Translation2d(x, y).rotateBy(pose.getRotation());
            out[i] = new Translation3d(
                    pose.getX() + inField.getX(), pose.getY() + inField.getY(), z);
        }
        return out;
    }

    private void updateShots() {
        if (!kicker.isFeeding() || fuelStored <= 0) {
            return;
        }

        shotAccumulator += 0.020 * SimConstants.SHOTS_PER_SECOND;
        while (shotAccumulator >= 1.0 && fuelStored > 0) {
            shotAccumulator -= 1.0;
            fuelStored--;
            if (drivebase.isInAllianceZone()) {
                hubShotsFired++;
            }
            launchOne();
        }
    }

    /**
     * The shot the robot would take right now, sampled as a ballistic arc from the TURRET
     * rather than from the robot origin. Add it in AdvantageScope as a Trajectory and the line
     * stems from the shooter itself, which is both better looking and more honest than a
     * vision-target line anchored at the robot centre.
     */
    private Pose3d[] aimTrajectory() {
        Translation2d turretPos = drivebase.getTurretFieldPosition();
        Translation2d turretVel = drivebase.getTurretFieldVelocity();
        double exitRad = Math.toRadians(90.0 - hood.getAngleDegrees());
        double yawRad = Math.toRadians(turret.getContinuousDegrees())
                + drivebase.getPose().getRotation().getRadians();
        double speed = ballSpeedMetersPerSecond(RPSToRPM(shooter.getRPS()));
        if (speed < 0.5) {
            return new Pose3d[0];
        }

        double horizontal = speed * Math.cos(exitRad);
        double vx = horizontal * Math.cos(yawRad) + turretVel.getX();
        double vy = horizontal * Math.sin(yawRad) + turretVel.getY();
        double vz = speed * Math.sin(exitRad);
        double x = turretPos.getX();
        double y = turretPos.getY();
        double z = DrivebaseConstants.SHOOTER_HEIGHT_M;

        java.util.List<Pose3d> points = new java.util.ArrayList<>();
        double step = 0.04;
        for (int i = 0; i < 60 && z > 0.0; i++) {
            points.add(new Pose3d(x, y, z, Rotation3d.kZero));
            double vh = Math.hypot(vx, vy);
            double kM = ShooterConstants.LINEAR_DRAG_K * ShooterConstants.MAGNUS_LIFT_RATIO;
            double ax = -ShooterConstants.LINEAR_DRAG_K * vx;
            double ay = -ShooterConstants.LINEAR_DRAG_K * vy;
            double az = -ShooterConstants.LINEAR_DRAG_K * vz - 9.81;
            if (vh > 1e-6) {
                ax += -kM * vz * vx / vh;
                ay += -kM * vz * vy / vh;
                az += kM * vh;
            }
            x += vx * step;
            y += vy * step;
            z += vz * step;
            vx += ax * step;
            vy += ay * step;
            vz += az * step;
        }
        return points.toArray(new Pose3d[0]);
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
