package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.PushoutConstants;

import static frc.robot.utils.utils.*;

import org.littletonrobotics.junction.Logger;

public class Pushout extends SubsystemBase {

    private TalonFX PushoutMotor = new TalonFX(PushoutConstants.PUSHOUT_ID);

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final CoastOut coastRequest = new CoastOut();

    private final CurrentLimitsConfigs currentLimits;

    public Pushout() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = PushoutConstants.PUSHOUT_STATOR_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = PushoutConstants.p;
        config.Slot0.kI = PushoutConstants.i;
        config.Slot0.kD = PushoutConstants.d;
        config.Slot0.kS = PushoutConstants.s;
        config.Slot0.kV = PushoutConstants.v;
        config.Slot0.kA = PushoutConstants.a;
        config.MotionMagic.MotionMagicCruiseVelocity = PushoutConstants.PUSHOUT_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = PushoutConstants.PUSHOUT_ACCELERATION;

        PushoutMotor.getConfigurator().apply(config);
        currentLimits = config.CurrentLimits;
        trimCanBus(PushoutMotor);
        PushoutMotor.setPosition(0);
    }

    public void PushIntake() {
        // PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.PUSHOUT_EXTENDED_POS).withSlot(0));
        PushoutMotor.setControl(
                positionRequest.withPosition(PushoutConstants.PUSHOUT_EXTENDED_POS).withSlot(0).withEnableFOC(Constants.USE_FOC));
    }

    public void RetractIntake() {
        // PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.PUSHOUT_RETRACTED_POS).withSlot(0));
        PushoutMotor.setControl(
                positionRequest.withPosition(PushoutConstants.PUSHOUT_RETRACTED_POS).withSlot(0).withEnableFOC(Constants.USE_FOC));
    }

    public void FullyRetract() {
        // PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.FULLY_RETRACTED_POS).withSlot(0));
        PushoutMotor.setControl(
                positionRequest.withPosition(PushoutConstants.FULLY_RETRACTED_POS).withSlot(0).withEnableFOC(Constants.USE_FOC));
    }

    public void ResetEncoder() {
        PushoutMotor.setPosition(0);
    }

    public void StopPushout() {
        // PushoutMotor.setControl(voltageRequest.withOutput(0));
        PushoutMotor.setControl(voltageRequest.withOutput(0).withEnableFOC(Constants.USE_FOC));
    }

    public void CoastPushout() {
        PushoutMotor.setControl(coastRequest);
    }

    public void PushoutDutyCycle(double output) {
        PushoutMotor.setControl(voltageRequest.withOutput(output).withEnableFOC(Constants.USE_FOC));
    }

    public void PushoutDutyCycleRetract(double output) {
        PushoutMotor.setControl(voltageRequest.withOutput(output).withEnableFOC(Constants.USE_FOC));
    }

    public double getPosition() {
        return PushoutMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtExtended() {
        return Math.abs(
                getPosition() - PushoutConstants.PUSHOUT_EXTENDED_POS) <= PushoutConstants.PUSHOUT_AT_TARGET_TOLERANCE;
    }

    private void setStatorLimit(double amps) {
        PushoutMotor.getConfigurator().apply(currentLimits.withStatorCurrentLimit(amps));
    }

    public Command CompliantPushCommand() {
        return this.run(this::PushIntake)
                .beforeStarting(() -> setStatorLimit(PushoutConstants.PUSHOUT_COMPLIANT_STATOR_LIMIT))
                .finallyDo(interrupted -> {
                    setStatorLimit(PushoutConstants.PUSHOUT_STATOR_LIMIT);
                    CoastPushout();
                });
    }

    public Command PushoutDutyCycleCommand() {
        return this.run(() -> {
            PushoutDutyCycle(PushoutConstants.dutyExtendSpeed);
        }).finallyDo(interrupted -> StopPushout());
    }

    public Command PushoutDutyCycleRetractCommand() {
        return this.run(() -> {
            PushoutDutyCycleRetract(PushoutConstants.dutyRetractSpeed);
        }).finallyDo(interrupted -> StopPushout());
    }

    public Command PushoutDutyCycleCommand(double output) {
        return this.run(() -> {
            PushoutDutyCycle(output);
        }).finallyDo(interrupted -> StopPushout());
    }

    public Command PushoutDutyCycleRetractCommand(double output) {
        return this.run(() -> {
            PushoutDutyCycleRetract(output);
        }).finallyDo(interrupted -> StopPushout());
    }

    public Command PushCommand() {
        return this.run(() -> {
            PushIntake();
        });
        // .finallyDo(interrupted -> StopPushout());
    }

    public Command RetractCommand() {
        return this.runOnce(() -> {
            RetractIntake();
        });
    }

    public Command FullyRetractCommand() {
        return this.runOnce(() -> {
            FullyRetract();
        });
    }

    public Command ResetEncoderCommand() {
        return this.runOnce(() -> {
            ResetEncoder();
        });
    }

    public Command CheesyAgitation() {
        return PushoutDutyCycleRetractCommand(PushoutConstants.cheesySpeed);
    }

    private void goToPosition(double rotations) {
        double clamped = MathUtil.clamp(rotations,
                PushoutConstants.FULLY_RETRACTED_POS, PushoutConstants.PUSHOUT_EXTENDED_POS);
        PushoutMotor.setControl(positionRequest.withPosition(clamped).withSlot(0));
    }

    private boolean isNear(double rotations) {
        return Math.abs(getPosition() - rotations) <= PushoutConstants.PUSHOUT_AGITATE_TOLERANCE;
    }

    public Command AgitateCommand() {
        Command agitate = Commands.repeatingSequence(
                agitateTo(PushoutConstants.PUSHOUT_FLUSH_WITH_BUMPER_POS),
                agitateTo(PushoutConstants.PUSHOUT_EXTENDED_POS))
                .finallyDo(interrupted -> PushIntake());

        agitate.addRequirements(this);
        return agitate;
    }

    private Command agitateTo(double rotations) {
        return Commands.sequence(
                runOnce(() -> goToPosition(rotations)),
                Commands.waitUntil(() -> isNear(rotations))
                        .withTimeout(PushoutConstants.PUSHOUT_EXTEND_TIMEOUT),
                Commands.waitSeconds(PushoutConstants.PUSHOUT_AGITATE_WAIT));
    }

    public Command runDefaultCommand() {
        return this.run(() -> {
            StopPushout();
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Pushout/Position", PushoutMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Pushout/TargetPosition", positionRequest.Position);
        Logger.recordOutput("Pushout/Velocity", PushoutMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Pushout/Voltage", getAppliedVoltage(PushoutMotor));
        Logger.recordOutput("Pushout/CurrentDraw", getSupplyCurrent(PushoutMotor));
        Logger.recordOutput("Pushout/StatorCurrent", getStatorCurrent(PushoutMotor));
        logFOC("Pushout", PushoutMotor);
        Logger.recordOutput("Pushout/IsAtExtended", isAtExtended());
        Logger.recordOutput("Pushout/StatorLimit", currentLimits.StatorCurrentLimit);
    }
}