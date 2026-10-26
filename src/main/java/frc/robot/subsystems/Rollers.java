package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.RollersConstants;

import frc.robot.utils.JamDetector;

import static frc.robot.utils.utils.*;

import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

    private TalonFX RollersMotor = new TalonFX(RollersConstants.ROLLERS_ID);

   
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final JamDetector jamDetector = new JamDetector(RollersConstants.JAMCURRENT,
            RollersConstants.JAM_STALL_SPEED_FRACTION, RollersConstants.JAM_IGNORE_SECONDS,
            RollersConstants.JAM_DEBOUNCE_SECONDS, RollersConstants.JAM_REVERSE_SECONDS);


    public Rollers() {
        TalonFXConfiguration RollersConfig = new TalonFXConfiguration();
        RollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        RollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust we have to
        RollersConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        RollersConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        RollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        RollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        RollersConfig.Slot0.kP = RollersConstants.p;
        RollersConfig.Slot0.kI = RollersConstants.i;
        RollersConfig.Slot0.kD = RollersConstants.d;
        RollersConfig.Slot0.kS = RollersConstants.s;
        RollersConfig.Slot0.kV = RollersConstants.v; 
        RollersConfig.Slot0.kA = RollersConstants.a;
        RollersMotor.getConfigurator().apply(RollersConfig);
        RollersMotor.getStatorCurrent().setUpdateFrequency(50);
        trimCanBus(RollersMotor);
    }

    public void ReverseRollers() {
        // RollersMotor.setControl(velocityRequest.withVelocity(RollersConstants.REVERSE_ROLLERS_RPS).withSlot(0));
        // RollersMotor.setControl(dutyCycleRequest.withOutput(RollersConstants.REVERSE_ROLLERS_SPEED));
        RollersMotor.setControl(dutyCycleRequest.withOutput(RollersConstants.REVERSE_ROLLERS_SPEED).withEnableFOC(Constants.USE_FOC));
    }

    public void RollersToConveyor() {
        RollersToConveyor(false);
    }

    public void RollersToConveyor(boolean passing) {
        double rps = passing ? RollersConstants.ROLLERS_PASSING_RPS : RollersConstants.ROLLERS_SCORING_RPS;
        RollersMotor.setControl(velocityRequest.withVelocity(rps).withEnableFOC(Constants.USE_FOC));
    }

    public void RunRollers(boolean passing)
    {
        double targetRPS = passing ? RollersConstants.ROLLERS_PASSING_RPS : RollersConstants.ROLLERS_SCORING_RPS;
        double speedFraction = Math.abs(RollersMotor.getVelocity().getValueAsDouble()) / targetRPS;
        if(jamDetector.shouldReverse(getStatorCurrent(RollersMotor), speedFraction))
        {
            ReverseRollers();
        }
        else
        {
            RollersToConveyor(passing);
        }
    }

    public boolean isUnjamming() {
        return jamDetector.isReversing();
    }

    public void ReverseForUnjam() {
        jamDetector.reset();
        ReverseRollers();
    }

    public void stopRollers() {
        jamDetector.reset();
        // RollersMotor.setControl(dutyCycleRequest.withOutput(0.0));
        RollersMotor.setControl(dutyCycleRequest.withOutput(0.0).withEnableFOC(Constants.USE_FOC));
    }


    public Command runRollersToConveyorCommand() {
        return this.run(() -> {
            RollersToConveyor();
        }).finallyDo(interrupted -> stopRollers());
    }

    public Command RunRollersCommand() {
        return this.run(() -> {
            RunRollers(false);
        }).finallyDo(interrupted -> stopRollers());
    }

    public Command runReverseRollersCommand() {
        return this.run(() -> {
            ReverseRollers();
        }).finallyDo(interrupted -> stopRollers());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Rollers/TargetRPS", getTargetRPS(RollersMotor));
        Logger.recordOutput("Rollers/Voltage", getAppliedVoltage(RollersMotor));
        Logger.recordOutput("Rollers/SupplyCurrentDraw", getSupplyCurrent(RollersMotor));
        Logger.recordOutput("Rollers/StatorCurrentDraw", getStatorCurrent(RollersMotor));
        Logger.recordOutput("Rollers/RPS", RollersMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rollers/Unjamming", jamDetector.isReversing());

        logFOC("Rollers", RollersMotor);
    }
}