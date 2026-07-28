package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.RollersConstants;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

    private TalonFX RollersMotor = new TalonFX(RollersConstants.ROLLERS_ID);

   
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Rollers() {
        TalonFXConfiguration RollersConfig = new TalonFXConfiguration();
        RollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        RollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust we have to
        RollersConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        RollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        RollersConfig.Slot0.kP = RollersConstants.p;
        RollersConfig.Slot0.kI = RollersConstants.i;
        RollersConfig.Slot0.kD = RollersConstants.d;
        RollersConfig.Slot0.kS = RollersConstants.s;
        RollersConfig.Slot0.kV = RollersConstants.v; 
        RollersConfig.Slot0.kA = RollersConstants.a;
        RollersMotor.getConfigurator().apply(RollersConfig);
    }

    public void ReverseRollers() {
        // RollersMotor.setControl(velocityRequest.withVelocity(RollersConstants.REVERSE_ROLLERS_RPS).withSlot(0));
        RollersMotor.setControl(dutyCycleRequest.withOutput(RollersConstants.REVERSE_ROLLERS_SPEED));
    }

    public void RollersToConveyor() {
        // RollersMotor.setControl(velocityRequest.withVelocity(RollersConstants.ROLLERS_RPS).withSlot(0));
        RollersMotor.setControl(dutyCycleRequest.withOutput(RollersConstants.ROLLERS_SPEED));
    }

    

    public void stopRollers() {
        RollersMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }


    public Command runRollersToConveyorCommand() {
        return this.run(() -> {
            RollersToConveyor();
        }).finallyDo(interrupted -> stopRollers());
    }


    public Command runReverseRollersCommand() {
        return this.run(() -> {
            ReverseRollers();
        }).finallyDo(interrupted -> stopRollers());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Rollers/DesiredRPS", RollersMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rollers/Volts", RollersMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rollers/RPS", RollersMotor.getVelocity().getValueAsDouble());
    }
}