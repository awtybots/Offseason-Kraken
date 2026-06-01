package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import org.littletonrobotics.junction.Logger;

import frc.robot.Configs;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private SparkMax TurretMotor = new SparkMax(TurretConstants.TURRET_ID, MotorType.kBrushless);
    private SparkClosedLoopController TurretController = TurretMotor.getClosedLoopController();

    private SparkAbsoluteEncoder absoluteEncoder = TurretMotor.getAbsoluteEncoder();
    private RelativeEncoder relativeEncoder = TurretMotor.getEncoder();

    private int wrapCount = 0; // # of times its wrapped
    private double lastAbsolutePosition = 0.0; // last absolute encoder reading in degrees, used for tracking wraps

    public Turret() {
        TurretMotor.configure(Configs.TurretSubsystem.TurretMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        relativeEncoder.setPosition(absoluteEncoder.getPosition()); // set relative to absolute at startup
        lastAbsolutePosition = absoluteEncoder.getPosition();
    }

    public double getAbsoluteDegrees() {
        return absoluteEncoder.getPosition(); // returns 0-360, resets at boundary
    }

    public double getRelativeDegrees() {
        return relativeEncoder.getPosition(); // continuous, doesnt reset at boundary
    }

    public double getContinuousDegrees() { // ts gets the continuous angle without wrapping
        return wrapCount * 360.0 + getAbsoluteDegrees(); // makes the turret know where it is w/o losing track across the wrap boundaries
    }

    private void updateWrapCount() { // detects when the abs encoder crosses boundary and changes the wrap count
        double current = getAbsoluteDegrees(); // if the reading jumps 180 degrees, then it wrapped
        double delta = current - lastAbsolutePosition;

        if (delta < -180.0) {
            wrapCount++; // crossed 0 going forward (359 → 0)
        } else if (delta > 180.0) {
            wrapCount--; // crossed 0 going backward (0 → 359)
        }

        lastAbsolutePosition = current; // update for next loop
    }

    public boolean isAtCableLimit() { // true = turret hit its leash, needs to wrap around
        double continuous = getContinuousDegrees();
        return continuous >= TurretConstants.MAX_CONTINUOUS_DEGREES
                || continuous <= TurretConstants.MIN_CONTINUOUS_DEGREES;
    }

    public void resetWrapCount() { // call this when turret is known to be at home/zero
        wrapCount = 0;
        lastAbsolutePosition = getAbsoluteDegrees();
        relativeEncoder.setPosition(getAbsoluteDegrees()); // re-sync relative to absolute
    }

    public double angleToSetpoint(double targetDegrees) { // converts a field-relative angle to a safe turret setpoint
        double candidate = targetDegrees + wrapCount * 360.0; // normalize into the same lap as current position

        if (candidate > TurretConstants.MAX_CONTINUOUS_DEGREES) {
            candidate -= 360.0; // try one lap down
        } else if (candidate < TurretConstants.MIN_CONTINUOUS_DEGREES) {
            candidate += 360.0; // try one lap up
        }

        if (candidate > TurretConstants.MAX_CONTINUOUS_DEGREES
                || candidate < TurretConstants.MIN_CONTINUOUS_DEGREES) {
            return Double.NaN; // no valid position exists, caller needs to handle this
        }

        return candidate;
    }

    public void setAngle(double degrees) { // send turret to angle using relative encoder for precision
        TurretController.setSetpoint(degrees, ControlType.kPosition);
    }

    public void stopTurret() {
        TurretMotor.set(0);
    }

    public void manualDrive(double speed) {
        if (isAtCableLimit()) { // dont let it drive past the cable limit
            TurretMotor.set(0);
            return;
        }
        TurretMotor.set(speed);
    }

    public Command goToAngleCommand(double degrees) { // points turret to a field-relative angle, handles wrap automatically
        return this.run(() -> {
            double setpoint = angleToSetpoint(degrees);
            if (!Double.isNaN(setpoint)) { // only move if theres a valid position to go to
                setAngle(setpoint);
            }
        }).finallyDo(interrupted -> stopTurret());
    }

    public Command manualDriveCommand(double speed) {
        return this.run(() -> {
            manualDrive(speed); // cable limit check is inside manualDrive()
        }).finallyDo(interrupted -> stopTurret());
    }

    public Command resetWrapCountCommand() {
        return this.runOnce(() -> {
            resetWrapCount();
        });
    }

    public Command runDefaultCommand() {
        return this.run(() -> {
            stopTurret();
        });
    }

    @Override
    public void periodic() {
        updateWrapCount(); // must run every loop for wrap detection to work

        Logger.recordOutput("Turret/AbsoluteDegrees", getAbsoluteDegrees());
        Logger.recordOutput("Turret/RelativeDegrees", getRelativeDegrees());
        Logger.recordOutput("Turret/ContinuousDegrees", getContinuousDegrees());
        Logger.recordOutput("Turret/WrapCount", wrapCount);
        Logger.recordOutput("Turret/IsAtCableLimit", isAtCableLimit());
        Logger.recordOutput("Turret/AppliedVolts", TurretMotor.getAppliedOutput() * TurretMotor.getBusVoltage());
        Logger.recordOutput("Turret/StatorCurrent", TurretMotor.getOutputCurrent());
    }
}