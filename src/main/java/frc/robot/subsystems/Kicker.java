package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.KickerConstants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {

    private TalonFX KickerMotor = new TalonFX(KickerConstants.KICKER_ID);
    private TalonFX VerticalRollerMotor = new TalonFX(KickerConstants.VERT_ROLLER_ID);

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Kicker() {
        TalonFXConfiguration KickerConfig = new TalonFXConfiguration();
        KickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        KickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust if needed
        KickerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        KickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        KickerMotor.getConfigurator().apply(KickerConfig);


        TalonFXConfiguration VerticalRollerConfig = new TalonFXConfiguration();
        VerticalRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        VerticalRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        VerticalRollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        VerticalRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        VerticalRollerMotor.getConfigurator().apply(VerticalRollerConfig);
    }

    public void ReverseKicker() {
        VerticalRollerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.VERT_ROLLER_REVERSE_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_REVERSE_SPEED));
    }

    public void ConveyorToShooter() {
        VerticalRollerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.VERT_ROLLER_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
    }

    public void ClearBall() {
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
    }


    public void stopKicker() {
        KickerMotor.setControl(dutyCycleRequest.withOutput(0));
        VerticalRollerMotor.setControl(dutyCycleRequest.withOutput(0));
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
        Logger.recordOutput("Kicker/TopDutyCycle", KickerMotor.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerDutyCycle", VerticalRollerMotor.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("Kicker/TopVolts", KickerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerVolts", VerticalRollerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Kicker/TopRPS", KickerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerRPS", VerticalRollerMotor.getVelocity().getValueAsDouble());
    }
}