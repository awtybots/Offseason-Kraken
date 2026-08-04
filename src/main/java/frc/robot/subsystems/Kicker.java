package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;

// import frc.robot.Configs;
import frc.robot.Constants.KickerConstants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {

    private TalonFX KickerMotor = new TalonFX(KickerConstants.KICKER_ID);
    // private SparkMax VerticalRollerMotor = new SparkMax(KickerConstants.VERT_ROLLER_ID, MotorType.kBrushless);
    // private RelativeEncoder VertRollerEncoder = VerticalRollerMotor.getEncoder();
    // private SparkClosedLoopController VerticalRollerController = VerticalRollerMotor.getClosedLoopController(); 

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Kicker() {
        TalonFXConfiguration KickerConfig = new TalonFXConfiguration();
        KickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        KickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust if needed
        KickerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        KickerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        KickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        KickerMotor.getConfigurator().apply(KickerConfig);

        // VerticalRollerMotor.configure(Configs.KickerSubsystem.VertivalMotorConfig, ResetMode.kResetSafeParameters,
                // PersistMode.kPersistParameters);
    }

    public void ReverseKicker() {
        // VerticalRollerController.setSetpoint(KickerConstants.VERT_ROLLER_REVERSE_SPEED, ControlType.kMAXMotionVelocityControl);
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_REVERSE_SPEED));
    }

    public void ConveyorToShooter() {
        // VerticalRollerMotor.set(KickerConstants.VERT_ROLLER_SPEED);
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
    }

    public void ClearBall() {
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
    }


    public void stopKicker() {
        KickerMotor.setControl(dutyCycleRequest.withOutput(0));
        // VerticalRollerMotor.set(0.0);
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
        Logger.recordOutput("Kicker/TopVolts", KickerMotor.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Kicker/VerticalRollerVolts", VerticalRollerMotor.getBusVoltage());
        Logger.recordOutput("Kicker/TopRPS", KickerMotor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Kicker/VerticalRollerRPS", VertRollerEncoder.getVelocity());
    }
}