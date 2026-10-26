package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import frc.robot.Configs;
import frc.robot.Constants.HoodConstants;
import frc.robot.utils.StallHomer;

import static frc.robot.utils.utils.*;

public class Hood extends SubsystemBase {

    private SparkMax HoodMotor = new SparkMax(HoodConstants.HOOD_ID, MotorType.kBrushless);
    private SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();
    private RelativeEncoder HoodEncoder = HoodMotor.getEncoder();

    private double currentTargetDegrees = HoodConstants.HOOD_MIN_DEGREES; // tracks last commanded angle, used for isAtAngle check
    private final StallHomer homer = new StallHomer(HoodConstants.HOMING_STALL_RPM,
            HoodConstants.HOMING_MIN_SECONDS, HoodConstants.HOMING_SETTLE_SECONDS, HoodConstants.HOMING_TIMEOUT_SECONDS);

    public Hood() {
        HoodMotor.configure(Configs.HoodSubsystem.HoodMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        HoodEncoder.setPosition(degreesToRotations(HoodConstants.HOOD_MIN_DEGREES)); // set to 21 which is start pos
    }

    private double degreesToRotations(double degrees) { // hood degrees to motor rotations taking gear ratio into account
        return (degrees / 360.0) * HoodConstants.GEAR_RATIO;
    }

    private double rotationsToDegrees(double rotations) { // opposite of above
        return (rotations / HoodConstants.GEAR_RATIO) * 360.0;
    }

    public double getTargetDegrees() {
        return currentTargetDegrees;
    }

    public double getAngleDegrees() {
        return rotationsToDegrees(HoodEncoder.getPosition());
    }


    public boolean isAtAngle() { // true = hood is within tolerance of its last commanded angle
        return homer.isHomed()
                && Math.abs(getAngleDegrees() - currentTargetDegrees) <= HoodConstants.ANGLE_TOLERANCE_DEGREES;
    }

    public void setAngle(double degrees) { // ensures is within limits and sends to controller
        double clamped = Math.max(HoodConstants.HOOD_MIN_DEGREES, Math.min(HoodConstants.HOOD_MAX_DEGREES, degrees));
        currentTargetDegrees = clamped; // track target so isAtAngle can check it
        if (!homer.isHoming()) {
            sendSetpoint();
        }
    }

    private void sendSetpoint() {
        HoodController.setSetpoint(degreesToRotations(currentTargetDegrees), ControlType.kPosition);
    }

    public void stopHood() {
        if (homer.isHoming()) {
            return;
        }
        HoodMotor.set(0);
    }

    public void goToMin() { // send hood to lowest position (20 deg)
        setAngle(HoodConstants.HOOD_MIN_DEGREES);
    }

    public void goToMax() { // send hood to highest position (40-45 deg, tune in constants)
        setAngle(HoodConstants.HOOD_MAX_DEGREES);
    }


    public Command setAngleCommand(double degrees) { // pass in any angle and hood goes there
        return this.run(() -> {
            setAngle(degrees);
        }).finallyDo(interrupted -> goToMin());
    }

    public Command goToMinCommand() {
        return this.run(() -> {
            goToMin();
        }).finallyDo(interrupted -> goToMin());
    }

    public Command goToMaxCommand() {
        return this.run(() -> {
            goToMax();
        }).finallyDo(interrupted -> goToMin());
    }

    public Command tuckCommand() {
        return this.run(() -> {
            goToMin(); // hold down position by default
        });
    }

    public Command HomeCommand() {
        return this.runOnce(homer::rehome);
    }

    @Override
    public void periodic() {
        switch (homer.update(DriverStation.isEnabled(), HoodEncoder.getVelocity())) {
            case DRIVE -> HoodMotor.set(-HoodConstants.HOMING_DUTY);
            case ZERO -> {
                HoodEncoder.setPosition(degreesToRotations(HoodConstants.HOOD_MIN_DEGREES));
                sendSetpoint();
            }
            case GAVE_UP -> sendSetpoint();
            case IDLE -> {
            }
        }
        Logger.recordOutput("Hood/AngleDegrees", getAngleDegrees());
        Logger.recordOutput("Hood/TargetDegrees", currentTargetDegrees);
        Logger.recordOutput("Hood/IsAtAngle", isAtAngle());
        Logger.recordOutput("Hood/MotorRotations", HoodEncoder.getPosition());
        Logger.recordOutput("Hood/Voltage", getAppliedVoltage(HoodMotor));
        Logger.recordOutput("Hood/StatorCurrent", getStatorCurrent(HoodMotor));
        Logger.recordOutput("Hood/CurrentDraw", getSupplyCurrent(HoodMotor));
        Logger.recordOutput("Hood/Homed", homer.isHomed());
        Logger.recordOutput("Hood/Homing", homer.isHoming());
        Logger.recordOutput("Hood/HomingTimedOut", homer.timedOut());
    }
}