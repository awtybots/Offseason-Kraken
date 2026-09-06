package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.PushoutConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.utils.utils;

public class Pushout extends SubsystemBase {

    private TalonFX PushoutMotor = new TalonFX(PushoutConstants.PUSHOUT_ID);

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Pushout() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = PushoutConstants.p;
        config.Slot0.kI = PushoutConstants.i;
        config.Slot0.kD = PushoutConstants.d;
        config.Slot0.kS = PushoutConstants.s;
        config.Slot0.kV = PushoutConstants.v;
        config.Slot0.kA = PushoutConstants.a;
        
        PushoutMotor.getConfigurator().apply(config);
        PushoutMotor.setPosition(0);
    }

    public void PushIntake() {
        // PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.PUSHOUT_EXTENDED_POS).withSlot(0));
        PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.PUSHOUT_EXTENDED_POS).withSlot(0).withEnableFOC(true));
    }

    public void RetractIntake() {
        // PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.PUSHOUT_RETRACTED_POS).withSlot(0));
        PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.PUSHOUT_RETRACTED_POS).withSlot(0).withEnableFOC(true));
    }

    public void FullyRetract() {
        // PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.FULLY_RETRACTED_POS).withSlot(0));
        PushoutMotor.setControl(positionRequest.withPosition(PushoutConstants.FULLY_RETRACTED_POS).withSlot(0).withEnableFOC(true));
    }

    public void ResetEncoder() {
        PushoutMotor.setPosition(0);
    }

    public void StopPushout() {
        // PushoutMotor.setControl(voltageRequest.withOutput(0));
        PushoutMotor.setControl(voltageRequest.withOutput(0).withEnableFOC(true));
    }

    public void PushoutDutyCycle() {
        // PushoutMotor.setControl(voltageRequest.withOutput(8.0));
        PushoutMotor.setControl(voltageRequest.withOutput(8.0).withEnableFOC(true));
    }

    public void PushoutDutyCycleRetract() {
        // PushoutMotor.setControl(voltageRequest.withOutput(-8.0));
        PushoutMotor.setControl(voltageRequest.withOutput(-8.0).withEnableFOC(true));
    }

    public void AgitateStep(boolean retract) {
        double extendedPos = PushoutConstants.PUSHOUT_EXTENDED_POS;
        double agitatePos = extendedPos - (extendedPos * 0.25);
        // PushoutMotor.setControl(positionRequest
        //         .withPosition(retract ? agitatePos : extendedPos)
        //         .withSlot(0));
        PushoutMotor.setControl(positionRequest
                .withPosition(retract ? agitatePos : extendedPos)
                .withSlot(0)
                .withEnableFOC(true));
    }

    public Command PushoutDutyCycleCommand() {
        return this.run(() -> {
            PushoutDutyCycle();
        }).finallyDo(interrupted -> StopPushout());
    }

    public Command PushoutDutyCycleRetractCommand() {
        return this.run(() -> {
            PushoutDutyCycleRetract();
        }).finallyDo(interrupted -> StopPushout());
    }

    public Command PushCommand() {
        return this.run(() -> {
            PushIntake();
        }).finallyDo(interrupted -> StopPushout());
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

    public Command AgitateCommand() {
        final double[] pullPositions = { 12.5, 10, 7, 5, 3 };
        final double[] pushPositions = { 15, 13.5, 10, 8.5, 6 };
        final double finalPos = 4;
        final double waitTime = PushoutConstants.PUSHOUT_AGITATE_WAIT;
        final double waitBetween = PushoutConstants.PUSHOUT_BETWEEN;

        Command agitate = Commands.sequence(
                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[0]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[0]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[0]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[0]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                Commands.waitSeconds(waitBetween),

                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[1]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[1]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[1]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[1]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                Commands.waitSeconds(waitBetween),

                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[2]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[2]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[2]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[2]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                Commands.waitSeconds(waitBetween),

                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[3]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[3]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[3]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[3]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                Commands.waitSeconds(waitBetween),

                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[4]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pullPositions[4]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[4]).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(pushPositions[4]).withSlot(0).withEnableFOC(true))),
                Commands.waitSeconds(waitTime),
                Commands.waitSeconds(waitBetween),

                // runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(finalPos).withSlot(0))),
                runOnce(() -> PushoutMotor.setControl(positionRequest.withPosition(finalPos).withSlot(0).withEnableFOC(true))),
                Commands.idle(this)

        ).finallyDo(interrupted -> PushIntake());

        agitate.addRequirements(this);
        return agitate;
    }

    public Command AgitateWhileIntakingCommand() {
        return Commands.repeatingSequence(
                runOnce(() -> AgitateStep(true)),
                Commands.waitSeconds(PushoutConstants.PUSHOUT_AGITATE_WAIT),
                runOnce(() -> AgitateStep(false)),
                Commands.waitSeconds(PushoutConstants.PUSHOUT_AGITATE_WAIT)
        ).finallyDo(interrupted -> PushIntake());
    }

    public Command runDefaultCommand() {
        return this.run(() -> {
            StopPushout();
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Pushout/Position", PushoutMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Pushout/Velocity", PushoutMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Pushout/AppliedVolts", PushoutMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Pushout/StatorCurrent", PushoutMotor.getStatorCurrent().getValueAsDouble());

        utils.logFOC("Pushout", PushoutMotor);
    }
}