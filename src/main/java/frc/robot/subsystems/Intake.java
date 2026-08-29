package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private double desiredPercent = 0.0;

    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_ID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Intake() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 120.0;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.Slot0.kP = IntakeConstants.p;
        intakeConfig.Slot0.kI = IntakeConstants.i;
        intakeConfig.Slot0.kD = IntakeConstants.d;
        intakeConfig.Slot0.kS = IntakeConstants.s;
        intakeConfig.Slot0.kV = IntakeConstants.v; 
        intakeConfig.Slot0.kA = IntakeConstants.a;
        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void runOuttake() {
        double rps = IntakeConstants.OUTTAKE_RPS; 
        intakeMotor.setControl(velocityRequest.withVelocity(rps).withSlot(0));
    }

    public void runIntake() {
        double rps = IntakeConstants.INTAKE_RPS; 
        intakeMotor.setControl(velocityRequest.withVelocity(rps).withSlot(0));
    }

    public void stopIntake() {
        desiredPercent = 0.0;
        intakeMotor.setControl(dutyCycleRequest.withOutput(0));
    }
  
    public Command runIntakeCommand() {
        return this.run(() -> {
            runIntake();
        }).finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        return this.run(() -> {
            runOuttake();
        }).finallyDo(interrupted -> stopIntake());
    }

    public Command stopIntakeCommand() {
        return this.run(() -> {
            stopIntake();
        });
    }

    public Command runDefaultCommand() {
        return stopIntakeCommand();
    }

    @Override
    public void periodic() {
        double leftRPS = intakeMotor.getVelocity().getValueAsDouble();

        Logger.recordOutput("Intake/DesiredPercent", desiredPercent);
        Logger.recordOutput("Intake/AppliedVolts", intakeMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Intake/LeftRPS", leftRPS);
        Logger.recordOutput("Intake/TargetRPS", IntakeConstants.INTAKE_RPS);
    }
}