package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

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

    private int wrapCount = 0; // # of times the through bore has wrapped since the last resync
    private double lastAbsolutePosition = 0.0; // last abs encoder reading in encoder degrees, used for tracking wraps
    private double currentTargetDegrees = 0.0; // tracks last commanded angle, used for isAtAngle check

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

        resyncFromAbsolute(); // assumes the turret booted parked at the reference spot
    }

    public double getAbsoluteDegrees() { // through bore shaft angle, (-180, 180]. NOT the turret angle, it spins 10x
                                         // faster
        return turretAbsoluteEncoder.getPosition();
    }

    public double getRelativeDegrees() {
        return rotationsToDegrees(turretRelativeEncoder.getPosition());
    }

    public double getContinuousDegrees() { // actual turret angle, unwrapped, 0 = robot forward
        double degreesFromReference = (wrapCount * 360.0 + getAbsoluteDegrees())
                / TurretConstants.ABSOLUTE_ENCODER_RATIO;
        return TurretConstants.REFERENCE_TURRET_DEGREES + degreesFromReference;
    }

    private double degreesToRotations(double degrees) { // gets motor rotations from the desired angle accounting for
                                                        // the gear ratio
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    private double rotationsToDegrees(double rotations) { // same but backwards
        return (rotations / TurretConstants.GEAR_RATIO) * 360.0;
    }

    private void updateWrapCount() { // detects when abs encoder crosses boundary and changes the wrap count
        double current = getAbsoluteDegrees(); // if the reading jumps 180 degrees then it wrapped itself
        double delta = current - lastAbsolutePosition;

        if (delta < -180.0) {
            wrapCount++; // crossed the seam going forward from 180 to -180
        } else if (delta > 180.0) {
            wrapCount--; // crossed the seam going backward from -180 to 180
        }

        lastAbsolutePosition = current; // update for next loop
    }

    public boolean isAtCableLimit() {
        double continuous = getContinuousDegrees();
        return continuous >= TurretConstants.MAX_CONTINUOUS_DEGREES
                || continuous <= TurretConstants.MIN_CONTINUOUS_DEGREES;
    }

     // true = turret is within tolerance of its last commanded angle
 public boolean isAtAngle() {
    return Math.abs(getContinuousDegrees() - currentTargetDegrees) <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
}

    // only call this with the turret parked at the reference spot. the through bore
    // cant tell
    // which 36 degree window its in, so this is us promising it that its in the
    // reference one.
    public void resyncFromAbsolute() {
        wrapCount = 0;
        lastAbsolutePosition = getAbsoluteDegrees(); // seed from where it actually is, otherwise the next loop fakes a
                                                     // wrap
        turretRelativeEncoder.setPosition(degreesToRotations(getContinuousDegrees()));
    }

    public double angleToSetpoint(double targetDegrees) { // converts angle to setpoint
        double current = getContinuousDegrees();
        double base = MathUtil.inputModulus(targetDegrees, -180.0, 180.0);

        double best = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;

        for (int lap = -1; lap <= 1; lap++) { // same heading is reachable at up to 2 laps in a 320 degree range
            double candidate = base + lap * 360.0;

            if (candidate > TurretConstants.MAX_CONTINUOUS_DEGREES
                    || candidate < TurretConstants.MIN_CONTINUOUS_DEGREES) {
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

    public boolean isTargetReachable() {
        return targetReachable;
    }

    /**
     * Command toward a field-derived angle, clamping into the cable range.
     * Returns false if the requested angle was outside that range.
     */
    public boolean setAngleClamped(double targetDegrees) {
        double setpoint = angleToSetpoint(targetDegrees);
        if (Double.isNaN(setpoint)) {
            double base = MathUtil.inputModulus(targetDegrees, -180.0, 180.0);
            setpoint = MathUtil.clamp(base, TurretConstants.MIN_CONTINUOUS_DEGREES,
                    TurretConstants.MAX_CONTINUOUS_DEGREES);
            targetReachable = false;
        } else {
            targetReachable = true;
        }
        setAngle(setpoint);
        return targetReachable;
    }

    public void setAngle(double degrees) { // send turret to angle in degrees using position control
        currentTargetDegrees = degrees; // track target so isAtAngle can check it
        // kPosition, not kMAXMotionPositionControl. Two reasons:
        // 1. TurretMotorConfig never sets a maxMotion block, and we configure with
        //    kResetSafeParameters, which wipes whatever was stored on the SPARK. So
        //    the motion profile ran on the controller's own defaults - nothing this
        //    code chose - and a zero cruise velocity there means no motion at all.
        // 2. MAXMotion is the wrong mode for a continuously moving setpoint anyway;
        //    it re-plans a deceleration ramp every loop. The hood already uses
        //    kPosition. If a trapezoid is wanted later, configure maxMotion first.
        turretController.setSetpoint(degreesToRotations(degrees), ControlType.kPosition);
    }

    public void stopTurret() {
        TurretMotor.set(0);
    }

    public void manualDrive(double speed) {
        if (isAtCableLimit()) { // dont let it pull its leash more than possible
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
        updateWrapCount(); // must run every loop for it to not blow up

        Logger.recordOutput("Turret/AbsoluteDegrees", getAbsoluteDegrees());
        Logger.recordOutput("Turret/AbsoluteDegPerSec", turretAbsoluteEncoder.getVelocity());
        Logger.recordOutput("Turret/RelativeDegrees", getRelativeDegrees());
        Logger.recordOutput("Turret/ContinuousDegrees", getContinuousDegrees());
        Logger.recordOutput("Turret/WrapCount", wrapCount);
        Logger.recordOutput("Turret/IsAtAngle", isAtAngle());
        Logger.recordOutput("Turret/IsAtCableLimit", isAtCableLimit());
        Logger.recordOutput("Turret/TargetDegrees", currentTargetDegrees);
        Logger.recordOutput("Turret/AppliedVolts", TurretMotor.getBusVoltage());
        Logger.recordOutput("Turret/Current", TurretMotor.getOutputCurrent());
        Logger.recordOutput("Turret/MotorRotations", turretRelativeEncoder.getPosition());
        Logger.recordOutput("Turret/FrameDisagreementDeg", getContinuousDegrees() - getRelativeDegrees());
    }
}