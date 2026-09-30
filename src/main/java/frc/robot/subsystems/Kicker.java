package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.utils.JamDetector;
import static frc.robot.utils.utils.*;


@SuppressWarnings("unused")
public class Kicker extends SubsystemBase {

    private TalonFX KickerMotor = new TalonFX(KickerConstants.KICKER_ID);
    private SparkMax VerticalRollerMotor = new SparkMax(KickerConstants.VERT_ROLLER_ID, MotorType.kBrushless);
    private RelativeEncoder VertRollerEncoder = VerticalRollerMotor.getEncoder();
    private SparkClosedLoopController VerticalRollerController = VerticalRollerMotor.getClosedLoopController();

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private double vertRollerTargetRPM = 0.0;

    private final JamDetector jamDetector = new JamDetector(KickerConstants.JAMCURRENT,
            KickerConstants.JAM_IGNORE_SECONDS, KickerConstants.JAM_DEBOUNCE_SECONDS,
            KickerConstants.JAM_REVERSE_SECONDS);

    public Kicker() {
        TalonFXConfiguration KickerConfig = new TalonFXConfiguration();
        KickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        KickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        KickerConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        KickerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        KickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        KickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        KickerConfig.Slot0.kP = KickerConstants.p;
        KickerConfig.Slot0.kI = KickerConstants.i;
        KickerConfig.Slot0.kD = KickerConstants.d;
        KickerConfig.Slot0.kS = KickerConstants.s;
        KickerConfig.Slot0.kV = KickerConstants.v;
        KickerConfig.Slot0.kA = KickerConstants.a;
        KickerMotor.getConfigurator().apply(KickerConfig);
        trimCanBus(KickerMotor);

        VerticalRollerMotor.configure(Configs.KickerSubsystem.VerticalMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void ReverseKicker() {
        vertRollerTargetRPM = 0.0;
        VerticalRollerMotor.set(KickerConstants.VERT_ROLLER_REVERSE_SPEED);
        // KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_REVERSE_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_REVERSE_SPEED).withEnableFOC(Constants.USE_FOC));
    }
 public void ReverseVerticalRoller(double surfaceMps) {
        double feederRPS = surfaceMps / (Math.PI * KickerConstants.FEEDER_WHEEL_DIAMETER_M) * KickerConstants.FEEDER_GEAR_RATIO;
        vertRollerTargetRPM = KickerConstants.VERT_ROLLER_REVERSE_SPEED;
        VerticalRollerController.setSetpoint(KickerConstants.VERT_ROLLER_REVERSE_SPEED, ControlType.kDutyCycle);
        KickerMotor.setControl(velocityRequest.withVelocity(feederRPS).withEnableFOC(Constants.USE_FOC));
    }


    public void RunKicker() {
        feedAtSurfaceSpeed(KickerConstants.FEEDER_MIN_SURFACE_MPS);
    }

    public void RunKicker(double shooterRPM) {
        double shooterSurfaceMps = RPMToRPS(shooterRPM) * 2 * Math.PI * ShooterConstants.ROLLER_RADIUS_BOTTOM_M;
        feedAtSurfaceSpeed(Math.max(KickerConstants.FEEDER_SHOOTER_SURFACE_RATIO * shooterSurfaceMps,
                KickerConstants.FEEDER_MIN_SURFACE_MPS));
    }

    private void feedAtSurfaceSpeed(double surfaceMps) {
        double feederRPS = surfaceMps / (Math.PI * KickerConstants.FEEDER_WHEEL_DIAMETER_M) * KickerConstants.FEEDER_GEAR_RATIO;
        vertRollerTargetRPM = KickerConstants.VERT_ROLLER_RPM;
        VerticalRollerController.setSetpoint(KickerConstants.VERT_ROLLER_RPM, ControlType.kVelocity);
        KickerMotor.setControl(velocityRequest.withVelocity(feederRPS).withEnableFOC(Constants.USE_FOC));
    }

    public void ConveyorToShooter() {
        if(jamDetector.shouldReverse(getStatorCurrent(VerticalRollerMotor)))
        {
            ReverseVerticalRoller(KickerConstants.FEEDER_MIN_SURFACE_MPS);
        }
        else
        {
            RunKicker();
        }
    }

    public void ConveyorToShooter(double shooterRPM) {
        if(jamDetector.shouldReverse(getStatorCurrent(VerticalRollerMotor)))
        {
            ReverseVerticalRoller(KickerConstants.FEEDER_MIN_SURFACE_MPS);
        }
        else
        {
            RunKicker(shooterRPM);
        }
    }
    

    public void ClearBall() {
        // KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED));
        KickerMotor.setControl(dutyCycleRequest.withOutput(KickerConstants.KICKER_SPEED).withEnableFOC(Constants.USE_FOC));
    }


    public void stopKicker() {
        jamDetector.reset();
        vertRollerTargetRPM = 0.0;
        // KickerMotor.setControl(dutyCycleRequest.withOutput(0));
        KickerMotor.setControl(dutyCycleRequest.withOutput(0).withEnableFOC(Constants.USE_FOC));
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
        Logger.recordOutput("Kicker/KickerDutyCycle", getDutyCycle(KickerMotor));
        Logger.recordOutput("Kicker/KickerVoltage", getAppliedVoltage(KickerMotor));
        Logger.recordOutput("Kicker/VerticalRoller/Voltage", getAppliedVoltage(VerticalRollerMotor));
        Logger.recordOutput("Kicker/VerticalRoller/CurrentDraw", getSupplyCurrent(VerticalRollerMotor));
        Logger.recordOutput("Kicker/KickerTargetRPS", getTargetRPS(KickerMotor));
        Logger.recordOutput("Kicker/KickerRPS", KickerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Kicker/VerticalRoller/TargetRPM", vertRollerTargetRPM);
        Logger.recordOutput("Kicker/VerticalRoller/RPM", VertRollerEncoder.getVelocity());
        Logger.recordOutput("Kicker/KickerCurrentDraw", getSupplyCurrent(KickerMotor));
        Logger.recordOutput("Kicker/VerticalRoller/StatorCurrent", getStatorCurrent(VerticalRollerMotor));
        Logger.recordOutput("Kicker/KickerStatorCurrent", getStatorCurrent(KickerMotor));

        Logger.recordOutput("Kicker/Unjamming", jamDetector.isReversing());
        logFOC("Kicker/Top", KickerMotor);
    }
}