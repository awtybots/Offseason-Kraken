package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ConveyorConstants;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

    private TalonFX ConveyorTopMotor = new TalonFX(ConveyorConstants.CONVEYOR_TOP_ID);
    private TalonFX ConveyorBottomMotor = new TalonFX(ConveyorConstants.CONVEYOR_BOTTOM_ID);

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Conveyor() {
        TalonFXConfiguration ConveyorConfig = new TalonFXConfiguration();
        ConveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ConveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        ConveyorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        ConveyorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ConveyorTopMotor.getConfigurator().apply(ConveyorConfig);

        ConveyorBottomMotor.getConfigurator().apply(ConveyorConfig);

        // Right follows Left, inverted
        ConveyorBottomMotor.setControl(new Follower(ConveyorTopMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void ReverseConveyor() {
        ConveyorTopMotor.setControl(dutyCycleRequest.withOutput(ConveyorConstants.CONVEYOR_REVERSE_SPEED));
    }

    public void HopperToShooter() {
        ConveyorTopMotor.setControl(dutyCycleRequest.withOutput(ConveyorConstants.CONVEYOR_SPEED));
    }


    public void stopConveyor() {
        ConveyorTopMotor.setControl(dutyCycleRequest.withOutput(0));
        // bottom follows for all the voids
    }
    
    public Command runDefaultCommand() {
        return this.run(() -> {
            stopConveyor();
        }).finallyDo(interrupted -> stopConveyor());
    }

    public Command ConveyorCommand() {
        return this.run(() -> {
            HopperToShooter();
        }).finallyDo(interrupted -> stopConveyor());
    }

    public Command ReverseConveyorCommand() {
        return this.run(() -> {
            ReverseConveyor();
        }).finallyDo(interrupted -> stopConveyor());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Conveyor/TopDutyCycle", ConveyorTopMotor.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("Conveyor/BottomDutyCycle", ConveyorBottomMotor.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("Conveyor/TopVolts", ConveyorTopMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Conveyor/BottomVolts", ConveyorBottomMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Conveyor/TopRPS", ConveyorTopMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Conveyor/BottomRPS", ConveyorBottomMotor.getVelocity().getValueAsDouble());
    }
}