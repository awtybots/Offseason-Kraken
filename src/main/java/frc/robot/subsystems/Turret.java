package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private TalonFX TurretMotor = new TalonFX(TurretConstants.TURRET_ID);
    private CANcoder absoluteEncoder = new CANcoder(TurretConstants.TURRET_CANCODER_ID); // always knows true angle even after power cycle

    private final PositionVoltage positionRequest = new PositionVoltage(0); // position control
    private final VoltageOut voltageRequest = new VoltageOut(0); // open loop for manual + stop

    private int wrapCount = 0; // # of times its wrapped
    private double lastAbsolutePosition = 0.0; // last abs encoder reading in degrees, used for tracking wraps
    private double currentTargetDegrees = 0.0; // tracks last commanded angle, used for isAtAngle check

    public Turret() {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_OFFSET; // tune so 0 is the center of the turret range
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // correct i think
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // makes range -0.5 to 0.5 rotations so basically just -180 to 180 degrees
        absoluteEncoder.getConfigurator().apply(cancoderConfig);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // hopefully correct
        motorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Slot0.kP = TurretConstants.p;
        motorConfig.Slot0.kI = TurretConstants.i;
        motorConfig.Slot0.kD = TurretConstants.d;
        motorConfig.Slot0.kS = TurretConstants.s;
        motorConfig.Slot0.kV = TurretConstants.v;
        motorConfig.Slot0.kA = TurretConstants.a;
        TurretMotor.getConfigurator().apply(motorConfig);

        TurretMotor.setPosition(degreesToRotations(getAbsoluteDegrees())); // makes relative equal to abs at beginning
        lastAbsolutePosition = getAbsoluteDegrees();
    }

    public double getAbsoluteDegrees() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    public double getRelativeDegrees() {
        return rotationsToDegrees(TurretMotor.getPosition().getValueAsDouble());
    }

    public double getContinuousDegrees() {
        return wrapCount * 360.0 + getAbsoluteDegrees();
    }

    private double degreesToRotations(double degrees) { // gets motor rotations from the desired angle accounting for the gear ratio
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    private double rotationsToDegrees(double rotations) { // same but backwards
        return (rotations / TurretConstants.GEAR_RATIO) * 360.0;
    }

    private void updateWrapCount() { // detects when abs encoder crosses boundary and changes the wrap count
        double current = getAbsoluteDegrees(); // if the reading jumps 180 degrees then it wrapped itself
        double delta = current - lastAbsolutePosition;

        if (delta < -180.0) {
            wrapCount++; // crossed 0 going forward from 359 to 0
        } else if (delta > 180.0) {
            wrapCount--; // crossed 0 going backward from 0 to 359
        }

        lastAbsolutePosition = current; // update for next loop
    }

    public boolean isAtCableLimit() {
        double continuous = getContinuousDegrees();
        return continuous >= TurretConstants.MAX_CONTINUOUS_DEGREES
                || continuous <= TurretConstants.MIN_CONTINUOUS_DEGREES;
    }

    public boolean isAtAngle() { // true = turret is within tolerance of its last commanded angle
        return Math.abs(getRelativeDegrees() - currentTargetDegrees) <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
    }

    public void zeroEncoder() { // call when turret is at center
        TurretMotor.setPosition(0);
        wrapCount = 0;
        lastAbsolutePosition = 0;
    }

    public double angleToSetpoint(double targetDegrees) { // converts angle to setpoint
        double candidate = targetDegrees + wrapCount * 360.0; // put into same lap as current position

        if (candidate > TurretConstants.MAX_CONTINUOUS_DEGREES) {
            candidate -= 360.0; // try one lap down
        } else if (candidate < TurretConstants.MIN_CONTINUOUS_DEGREES) {
            candidate += 360.0; // try one lap up
        }

        if (candidate > TurretConstants.MAX_CONTINUOUS_DEGREES
                || candidate < TurretConstants.MIN_CONTINUOUS_DEGREES) {
            return Double.NaN; // if no valid pos, it doesnt kill itself
        }

        return candidate;
    }

    public void setAngle(double degrees) { // send turret to angle in degrees using position control
        currentTargetDegrees = degrees; // track target so isAtAngle can check it
        TurretMotor.setControl(positionRequest.withPosition(degreesToRotations(degrees)).withSlot(0));
    }

    public void stopTurret() {
        TurretMotor.setControl(voltageRequest.withOutput(0));
    }

    public void manualDrive(double speed) {
        if (isAtCableLimit()) { // dont let it pull its leash more than possible
            TurretMotor.setControl(voltageRequest.withOutput(0));
            return;
        }
        TurretMotor.setControl(voltageRequest.withOutput(speed * 12.0)); // scale -1 to 1 → volts
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

    public Command zeroEncoderCommand() {
        return this.runOnce(() -> {
            zeroEncoder();
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
        Logger.recordOutput("Turret/RelativeDegrees", getRelativeDegrees());
        Logger.recordOutput("Turret/ContinuousDegrees", getContinuousDegrees());
        Logger.recordOutput("Turret/WrapCount", wrapCount);
        Logger.recordOutput("Turret/IsAtAngle", isAtAngle());
        Logger.recordOutput("Turret/IsAtCableLimit", isAtCableLimit());
        Logger.recordOutput("Turret/TargetDegrees", currentTargetDegrees);
        Logger.recordOutput("Turret/AppliedVolts", TurretMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Turret/StatorCurrent", TurretMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Turret/MotorRotations", TurretMotor.getPosition().getValueAsDouble());
    }
}