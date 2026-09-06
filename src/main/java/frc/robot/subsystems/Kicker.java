package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Configs;
import frc.robot.Constants.KickerConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.utils.utils;


@SuppressWarnings("unused")
public class Kicker extends SubsystemBase {

    private TalonFX KickerMotor = new TalonFX(KickerConstants.KICKER_ID);
    private SparkMax VerticalRollerMotor = new SparkMax(KickerConstants.VERT_ROLLER_ID, MotorType.kBrushless);
    private RelativeEncoder VertRollerEncoder = VerticalRollerMotor.getEncoder();
    // private SparkClosedLoopController VerticalRollerController = VerticalRollerMotor.getClosedLoopController(); 

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Kicker() {
        TalonFXConfiguration KickerConfig = new TalonFXConfiguration();
        KickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        KickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust if needed
        KickerConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        KickerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        KickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        KickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        KickerMotor.getConfigurator().apply(KickerConfig);

        VerticalRollerMotor.configure(Configs.KickerSubsystem.VerticalMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void ReverseKicker() {
        VerticalRollerMotor.set(KickerConstants.VERT_ROLLER_REVERSE_SPEED);
        // KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_REVERSE_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_REVERSE_SPEED).withEnableFOC(true));
    }

    public void ConveyorToShooter() {
        VerticalRollerMotor.set(KickerConstants.VERT_ROLLER_SPEED);
        // KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED).withEnableFOC(true));
    }

    public void ClearBall() {
        // KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED).withEnableFOC(true));
    }


    public void stopKicker() {
        // KickerMotor.setControl(dutyCycleRequest.withOutput(0));
        KickerMotor.setControl(dutyCycleRequest.withOutput(0).withEnableFOC(true));
        VerticalRollerMotor.set(0.0);
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
        Logger.recordOutput("Kicker/KickerDutyCycle", KickerMotor.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("Kicker/KickerVoltage", KickerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerVoltage", VerticalRollerMotor.getBusVoltage() * VerticalRollerMotor.getAppliedOutput());
        Logger.recordOutput("Kicker/VerticalRollerCurrentDraw", VerticalRollerMotor.getOutputCurrent() * VerticalRollerMotor.getAppliedOutput());
        Logger.recordOutput("Kicker/KickerRPS", KickerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRollerRPM", VertRollerEncoder.getVelocity());
        Logger.recordOutput("Kicker/KickerCurrentDraw", KickerMotor.getSupplyCurrent().getValueAsDouble());

         utils.logFOC("Kicker/Top", KickerMotor);
    }
}