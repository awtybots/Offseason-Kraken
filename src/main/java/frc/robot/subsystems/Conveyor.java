package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ConveyorConstants;
public class Conveyor extends SubsystemBase {

    private TalonFX ConveyorTopMotor = new TalonFX(ConveyorConstants.CONVEYOR_TOP_ID);
    private TalonFX ConveyorBottomMotor = new TalonFX(ConveyorConstants.CONVEYOR_BOTTOM_ID);

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Conveyor() {
        TalonFXConfiguration ConveyorConfig = new TalonFXConfiguration();
        ConveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ConveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        ConveyorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        ConveyorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        ConveyorConfig.Slot0.kP = ConveyorConstants.p;
        ConveyorConfig.Slot0.kI = ConveyorConstants.i;
        ConveyorConfig.Slot0.kD = ConveyorConstants.d;  
        ConveyorConfig.Slot0.kS = ConveyorConstants.s;
        ConveyorConfig.Slot0.kV = ConveyorConstants.v;
        ConveyorConfig.Slot0.kA = ConveyorConstants.a;

        ConveyorTopMotor.getConfigurator().apply(ConveyorConfig);
        ConveyorBottomMotor.getConfigurator().apply(ConveyorConfig);

        // Bottom follows top inverted
        ConveyorBottomMotor.setControl(new Follower(ConveyorTopMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void ReverseConveyor() {
        ConveyorTopMotor.setControl(velocityRequest.withVelocity(ConveyorConstants.CONVEYOR_REVERSE_RPS).withSlot(0));
    }

    public void HopperToShooter() {
        ConveyorTopMotor.setControl(velocityRequest.withVelocity(ConveyorConstants.CONVEYOR_RPS).withSlot(0));
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

    public Command HopperToShooterCommand() {
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