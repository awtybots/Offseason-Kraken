package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.KickerConstants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {

    private TalonFX KickerMotor = new TalonFX(KickerConstants.KICKER_ID);
    private TalonFX VerticalRollerMotor = new TalonFX(KickerConstants.VERT_ROLLER_ID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Kicker() {
        TalonFXConfiguration KickerConfig = new TalonFXConfiguration();
        KickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        KickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust if needed
        KickerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        KickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        KickerConfig.Slot0.kP = KickerConstants.p;
        KickerConfig.Slot0.kI = KickerConstants.i;
        KickerConfig.Slot0.kD = KickerConstants.d;
        KickerConfig.Slot0.kS = KickerConstants.s;
        KickerConfig.Slot0.kV = KickerConstants.v; 
        KickerConfig.Slot0.kA = KickerConstants.a;
        KickerMotor.getConfigurator().apply(KickerConfig);


        TalonFXConfiguration VerticalRollerConfig = new TalonFXConfiguration();
        VerticalRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VerticalRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        VerticalRollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        VerticalRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        VerticalRollerConfig.Slot0.kP = KickerConstants.p;
        VerticalRollerConfig.Slot0.kI = KickerConstants.i;
        VerticalRollerConfig.Slot0.kD = KickerConstants.d;
        VerticalRollerConfig.Slot0.kS = KickerConstants.s;
        VerticalRollerConfig.Slot0.kV = KickerConstants.v; 
        VerticalRollerConfig.Slot0.kA = KickerConstants.a;
        VerticalRollerMotor.getConfigurator().apply(VerticalRollerConfig);
    }

    public void ReverseKicker() {
        VerticalRollerMotor.setControl(velocityRequest.withVelocity(KickerConstants.VERT_ROLLER_REVERSE_RPS).withSlot(0));
        KickerMotor.setControl(velocityRequest.withVelocity(KickerConstants.KICKER_REVERSE_RPS).withSlot(0));
    }

    public void ConveyorToShooter() {
        VerticalRollerMotor.setControl(velocityRequest.withVelocity(KickerConstants.VERT_ROLLER_RPS).withSlot(0));
        KickerMotor.setControl(velocityRequest.withVelocity(KickerConstants.KICKER_RPS).withSlot(0));
    }

    public void ClearBall() {
        KickerMotor.setControl(velocityRequest.withVelocity(KickerConstants.KICKER_RPS).withSlot(0));
    }


    public void stopKicker() {
        KickerMotor.setControl(velocityRequest.withVelocity(0));
        VerticalRollerMotor.setControl(velocityRequest.withVelocity(0));
    }

    public Command runDefaultCommand() {
        return new RunCommand(() -> stopKicker(), this);
    }

    public Command KickerCommand() {
        return this.run(() -> {
            ConveyorToShooter();
        }).finallyDo(interrupted -> stopKicker());
    }

    public Command ClearBallCommand() {
        return this.run(() -> {
            ClearBall();
        }).finallyDo(interrupted -> stopKicker());
    }

    public Command ReverseKickerCommand() {
        return this.run(() -> {
            ReverseKicker();
        }).finallyDo(interrupted -> stopKicker());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Kicker/TopDesiredRPS", KickerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerDesiredRPS", VerticalRollerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Kicker/TopVolts", KickerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerVolts", VerticalRollerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Kicker/TopRPS", KickerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerRPS", VerticalRollerMotor.getVelocity().getValueAsDouble());
    }
}