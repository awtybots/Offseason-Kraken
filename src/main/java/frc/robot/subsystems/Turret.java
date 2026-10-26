package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import static frc.robot.utils.utils.*;

import org.littletonrobotics.junction.Logger;

import frc.robot.Configs;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    // private TalonFX TurretMotor = new TalonFX(TurretConstants.TURRET_ID);

    // private final PositionVoltage positionRequest = new PositionVoltage(0); //
    // position control
    // private final VoltageOut voltageRequest = new VoltageOut(0); // open loop for
    // manual + stop

    private SparkMax TurretMotor = new SparkMax(TurretConstants.TURRET_ID, MotorType.kBrushless);
    private SparkClosedLoopController turretController = TurretMotor.getClosedLoopController();
    private RelativeEncoder turretRelativeEncoder = TurretMotor.getEncoder();
    private AbsoluteEncoder turretAbsoluteEncoder = TurretMotor.getAbsoluteEncoder(); // REV Through Bore on the data
                                                                                      // port

    private double currentTargetDegrees = 0.0; // tracks last commanded angle, used for isAtAngle check
    private boolean setpointWasClamped = false; // last setAngle call hit a travel limit
    private double arbFFVolts = 0.0;
    private final Timer bootTimer = new Timer();
    private boolean seeded = false;
    private double seedOffsetDegrees = 0.0;
    private double lastKnownDegrees = 0.0;
    private int resetRecoveries = 0;
    private int implausibleLoops = 0;

    public Turret() {
        // TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // // hopefully correct
        // motorConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        // motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        // motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // motorConfig.Slot0.kP = TurretConstants.p;
        // motorConfig.Slot0.kI = TurretConstants.i;
        // motorConfig.Slot0.kD = TurretConstants.d;
        // motorConfig.Slot0.kS = TurretConstants.s;
        // motorConfig.Slot0.kV = TurretConstants.v;
        // motorConfig.Slot0.kA = TurretConstants.a;
        // TurretMotor.getConfigurator().apply(motorConfig);

        // TurretMotor.setPosition(degreesToRotations(getAbsoluteDegrees())); // makes
        // relative equal to abs at beginning
        // lastAbsolutePosition = getAbsoluteDegrees();
        TurretMotor.configure(Configs.TurretSubsystem.TurretMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        TurretMotor.clearFaults();

        bootTimer.start();
    }

    public double getAbsoluteDegrees() { // through bore shaft angle, (-180, 180]. NOT the turret angle, it spins 10x
                                         // faster
        return turretAbsoluteEncoder.getPosition();
    }

    public double getRelativeDegrees() {
        return rotationsToDegrees(turretRelativeEncoder.getPosition());
    }

    public double getContinuousDegrees() { // actual turret angle, unwrapped, 0 = robot forward
        return getRelativeDegrees();
    }

    public double throughBoreDegreesNear(double nearDegrees) {
        double ratio = TurretConstants.ABSOLUTE_ENCODER_RATIO;
        double absolute = getAbsoluteDegrees();
        long window = Math.round(
                ((nearDegrees - TurretConstants.REFERENCE_TURRET_DEGREES) * ratio - absolute) / 360.0);
        return TurretConstants.REFERENCE_TURRET_DEGREES + (window * 360.0 + absolute) / ratio;
    }

    private void seedNear(double nearDegrees) {
        double seed = throughBoreDegreesNear(nearDegrees);
        turretRelativeEncoder.setPosition(degreesToRotations(seed));
        lastKnownDegrees = seed;
        implausibleLoops = 0;
        seeded = true;
    }

    public boolean isSeeded() {
        return seeded;
    }

    private double turretDegreesPerSecond() {
        return turretRelativeEncoder.getVelocity() * 6.0 / TurretConstants.GEAR_RATIO;
    }

    private double degreesToRotations(double degrees) { // gets motor rotations from the desired angle accounting for
                                                        // the gear ratio
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    private double rotationsToDegrees(double rotations) { // same but backwards
        return (rotations / TurretConstants.GEAR_RATIO) * 360.0;
    }

    // Travel limits backed off from the hard stops by the safety margin. Every
    // setpoint this class issues is clamped into [softMin, softMax], so the closed
    // loop can never command the turret past a stop - and if it somehow starts
    // outside the range, the clamped setpoint pulls it back instead of freezing.
    private static double softMinDegrees() {
        return TurretConstants.MIN_CONTINUOUS_DEGREES + TurretConstants.CABLE_LIMIT_MARGIN_DEGREES;
    }

    private static double softMaxDegrees() {
        return TurretConstants.MAX_CONTINUOUS_DEGREES - TurretConstants.CABLE_LIMIT_MARGIN_DEGREES;
    }

    public boolean isAtCableLimit() {
        double continuous = getContinuousDegrees();
        return continuous >= softMaxDegrees() || continuous <= softMinDegrees();
    }

    /**
     * True if driving at {@code speed} would push the turret further past its travel
     * limit. Motion back toward the valid range is always allowed, so the turret can
     * never latch itself against a stop with no way to drive off it.
     *
     * <p>Assumes positive output raises {@link #getContinuousDegrees()}. That is the
     * same convention {@link #degreesToRotations} already bakes into the position
     * loop, so it is not a new assumption.
     */
    public boolean wouldExceedCableLimit(double speed) {
        double continuous = getContinuousDegrees();
        if (speed > 0 && continuous >= softMaxDegrees()) {
            return true;
        }
        if (speed < 0 && continuous <= softMinDegrees()) {
            return true;
        }
        return false;
    }

     // true = turret is within tolerance of its last commanded angle
 public boolean isAtAngle() {
    return seeded
            && Math.abs(getContinuousDegrees() - currentTargetDegrees) <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
}

    // only call this with the turret parked at the reference spot. the through bore
    // cant tell
    // which 36 degree window its in, so this is us promising it that its in the
    // reference one.
    public void resyncFromAbsolute() {
        seedNear(TurretConstants.REFERENCE_TURRET_DEGREES);
    }

    public double angleToSetpoint(double targetDegrees) { // converts angle to setpoint
        double current = getContinuousDegrees();
        double base = MathUtil.inputModulus(targetDegrees, -180.0, 180.0);

        double best = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;

        for (int lap = -1; lap <= 1; lap++) { // 615 deg 
            double candidate = base + lap * 360.0;

            if (candidate > softMaxDegrees() || candidate < softMinDegrees()) {
                continue; // outside the cable limit, cant go there
            }

            double error = Math.abs(candidate - current);
            if (error < bestError) { // take the shortest trip
                bestError = error;
                best = candidate;
            }
        }

        return best; // NaN if no valid pos, so it doesnt kill itself

    }

    private boolean targetReachable = true;

    /** Last angle actually commanded, after the travel clamp. */
    public double getTargetDegrees() {
        return currentTargetDegrees;
    }

    public boolean isTargetReachable() {
        return targetReachable;
    }

    /**
     * Command toward a field-derived angle, clamping into the cable range.
     * Returns false if the requested angle was outside that range.
     */
    public boolean setAngleClamped(double targetDegrees) {
        return setAngleClamped(targetDegrees, 0.0);
    }

    public boolean setAngleClamped(double targetDegrees, double feedforwardDegPerSec) {
        double setpoint = angleToSetpoint(targetDegrees);
        if (Double.isNaN(setpoint)) {
            double base = MathUtil.inputModulus(targetDegrees, -180.0, 180.0);
            setpoint = MathUtil.clamp(base, softMinDegrees(), softMaxDegrees());
            targetReachable = false;
        } else {
            targetReachable = true;
        }
        setAngle(setpoint, targetReachable ? feedforwardDegPerSec : 0.0);
        return targetReachable;
    }

    public void setAngle(double degrees) {
        setAngle(degrees, 0.0);
    }

    public void setAngle(double degrees, double feedforwardDegPerSec) { // send turret to angle in degrees using position control
        // Hard guard for the closed loop. A position setpoint inside the soft range
        // cannot command the turret into a stop no matter who calls this.
        double clamped = MathUtil.clamp(degrees, softMinDegrees(), softMaxDegrees());
        setpointWasClamped = clamped != degrees;
        currentTargetDegrees = clamped; // track target so isAtAngle can check it
        double maxVolts = TurretConstants.MAX_OUTPUT * 12.0;
        arbFFVolts = setpointWasClamped ? 0.0
                : MathUtil.clamp(feedforwardDegPerSec * TurretConstants.FF_VOLTS_PER_DEG_PER_SEC,
                        -maxVolts, maxVolts);
        // kPosition, not kMAXMotionPositionControl. Two reasons:
        // 1. TurretMotorConfig never sets a maxMotion block, and we configure with
        //    kResetSafeParameters, which wipes whatever was stored on the SPARK. So
        //    the motion profile ran on the controller's own defaults - nothing this
        //    code chose - and a zero cruise velocity there means no motion at all.
        // 2. MAXMotion is the wrong mode for a continuously moving setpoint anyway;
        //    it re-plans a deceleration ramp every loop. The hood already uses
        //    kPosition. If a trapezoid is wanted later, configure maxMotion first.
        turretController.setSetpoint(degreesToRotations(clamped), ControlType.kPosition,
                ClosedLoopSlot.kSlot0, arbFFVolts, ArbFFUnits.kVoltage);
    }

    public void stopTurret() {
        TurretMotor.set(0);
    }

    public void manualDrive(double speed) {
        if (wouldExceedCableLimit(speed)) { // dont let it pull its leash more than possible
            stopTurret();
            return;
        }
        TurretMotor.set(speed); // scale -1 to 1 → volts
    }

    public Command goToAngleCommand(double degrees) { // full go to angle cmd
        return this.run(() -> {
            double setpoint = angleToSetpoint(degrees);
            if (!Double.isNaN(setpoint)) { // only move if theres a valid pos
                setAngle(setpoint);
            }
        }).finallyDo(interrupted -> stopTurret());
    }

    public Command manualDriveCommand(double speed) {
        return this.run(() -> {
            manualDrive(speed);
        }).finallyDo(interrupted -> stopTurret());
    }

    public Command resyncEncoderCommand() {
        return this.runOnce(() -> {
            resyncFromAbsolute();
        });
    }

    public Command runDefaultCommand() {
        return this.run(() -> {
            stopTurret();
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Turret/StickyFaultBits", TurretMotor.getStickyFaults().rawBits);
        double plausibleJump = Math.abs(turretDegreesPerSecond()) * TurretConstants.TRACK_JUMP_WINDOW_SECONDS
                + TurretConstants.TRACK_JUMP_MARGIN_DEGREES;
        if (TurretMotor.getStickyWarnings().hasReset) {
            TurretMotor.clearFaults();
            if (seeded && Math.abs(getRelativeDegrees() - lastKnownDegrees) > plausibleJump) {
                seeded = false;
                seedOffsetDegrees = lastKnownDegrees;
                resetRecoveries++;
            }
        }
        if (!seeded && bootTimer.hasElapsed(TurretConstants.BOOT_RESYNC_DELAY_SECONDS)
                && Math.abs(turretDegreesPerSecond()) < TurretConstants.SEED_MAX_DEGREES_PER_SECOND) {
            seedNear(seedOffsetDegrees + getRelativeDegrees());
            seedOffsetDegrees = 0.0;
        }
        if (seeded) {
            double now = getContinuousDegrees();
            if (Math.abs(now - lastKnownDegrees) <= plausibleJump
                    || ++implausibleLoops > TurretConstants.TRACK_JUMP_ACCEPT_LOOPS) {
                lastKnownDegrees = now;
                implausibleLoops = 0;
            }
        }

        Logger.recordOutput("Turret/AbsoluteDegrees", getAbsoluteDegrees());
        Logger.recordOutput("Turret/AbsoluteDegPerSec", turretAbsoluteEncoder.getVelocity());
        Logger.recordOutput("Turret/RelativeDegrees", getRelativeDegrees());
        Logger.recordOutput("Turret/ContinuousDegrees", getContinuousDegrees());
        Logger.recordOutput("Turret/Seeded", seeded);
        Logger.recordOutput("Turret/ResetRecoveries", resetRecoveries);
        Logger.recordOutput("Turret/IsAtAngle", isAtAngle());
        Logger.recordOutput("Turret/IsAtCableLimit", isAtCableLimit());
        Logger.recordOutput("Turret/TargetDegrees", currentTargetDegrees);
        // getBusVoltage() alone is the battery rail at the Spark, not the motor output -
        // it read a flat ~12.4 V while the turret sat still. Applied output is the duty
        // cycle scaled by the bus.
        Logger.recordOutput("Turret/AppliedVolts", getAppliedVoltage(TurretMotor));
        Logger.recordOutput("Turret/CurrentDraw", getSupplyCurrent(TurretMotor));
        Logger.recordOutput("Turret/StatorCurrent", getStatorCurrent(TurretMotor));
        Logger.recordOutput("Turret/MotorRotations", turretRelativeEncoder.getPosition());
        Logger.recordOutput("Turret/FrameDisagreementDeg",
                throughBoreDegreesNear(getContinuousDegrees()) - getContinuousDegrees());
        Logger.recordOutput("Turret/SoftMinDegrees", softMinDegrees());
        Logger.recordOutput("Turret/SoftMaxDegrees", softMaxDegrees());
        Logger.recordOutput("Turret/SetpointWasClamped", setpointWasClamped);
        Logger.recordOutput("Turret/ArbFFVolts", arbFFVolts);
        Logger.recordOutput("Turret/IsTargetReachable", isTargetReachable());
    }
}