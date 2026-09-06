package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;
import static frc.robot.utils.utils.*;

public class Shooter extends SubsystemBase {

    private TalonFX ShooterRightMotor = new TalonFX(ShooterConstants.SHOOTER_R_ID);
    private TalonFX ShooterLeftMotor = new TalonFX(ShooterConstants.SHOOTER_L_ID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private double targetRPS = 0.0;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        ShooterRightMotor.setVoltage(voltage.in(Units.Volts));
                        ShooterLeftMotor.setVoltage(voltage.in(Units.Volts));
                    },
                    log -> {
                        log.motor("shooter-right")
                                .voltage(Units.Volts.of(ShooterRightMotor.getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Units.Rotations.of(ShooterRightMotor.getPosition().getValueAsDouble()))
                                .angularVelocity(Units.RotationsPerSecond.of(ShooterRightMotor.getVelocity().getValueAsDouble()));

                        log.motor("shooter-left")
                                .voltage(Units.Volts.of(ShooterLeftMotor.getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Units.Rotations.of(ShooterLeftMotor.getPosition().getValueAsDouble()))
                                .angularVelocity(Units.RotationsPerSecond.of(ShooterLeftMotor.getVelocity().getValueAsDouble()));
                    },
                    this));

    public Shooter() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        shooterConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.Slot0.kP = ShooterConstants.p;
        shooterConfig.Slot0.kI = ShooterConstants.i;
        shooterConfig.Slot0.kD = ShooterConstants.d;
        shooterConfig.Slot0.kS = ShooterConstants.s;
        shooterConfig.Slot0.kV = ShooterConstants.v;
        shooterConfig.Slot0.kA = ShooterConstants.a;
        ShooterRightMotor.getConfigurator().apply(shooterConfig);
        ShooterLeftMotor.getConfigurator().apply(shooterConfig);

        // Follow the right motor
        ShooterLeftMotor.setControl(new Follower(ShooterRightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // Against the live setpoint, not SHOOTER_SPEED. That constant is a 2400 RPM bench
    // value while the tables now command 3065-5734 RPM, so this read false at all match.
    public boolean isShooterFast() {
        if (targetRPS == 0.0) {
            return false;
        }
        return Math.abs(getRPS() - targetRPS) <= ShooterConstants.ERROR_MARGIN;
    }

    public double getRPS() {
        return (Math.abs(ShooterRightMotor.getVelocity().getValueAsDouble())
                + Math.abs(ShooterLeftMotor.getVelocity().getValueAsDouble())) / 2.0;
    }

    public boolean isShooterRunning() {
        double setpoint = Math.abs(targetRPS);
        boolean running = Math.abs(setpoint - getRPS()) < ShooterConstants.ERROR_MARGIN
                && setpoint != 0
                && setpoint != RPMToRPS(ShooterConstants.ALLIANCE_IDLE_RPM);
        SmartDashboard.putBoolean("Shooter Running", running);
        return running;
    }

    public void shootFuel() {
        SpeedUpShooter();
    }

    public void stopShooting() {
        targetRPS = 0.0;
        ShooterRightMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }

    public void SpeedUpShooter() {
        targetRPS = RPMToRPS(ShooterConstants.SHOOTER_SPEED);
        setTargetRPM(ShooterConstants.SHOOTER_SPEED);
    }

    public void setTargetRPM(double rpm) {
        targetRPS = RPMToRPS(rpm);
        ShooterRightMotor.setControl(velocityRequest.withVelocity(RPMToRPS(rpm)).withSlot(0));
    }

    public void ShooterPassing() {
        targetRPS = RPMToRPS(ShooterConstants.SHOOTER_PASSING_SPEED);
        setTargetRPM(ShooterConstants.SHOOTER_PASSING_SPEED);
    }

    public Command setAllianceIdle() {
        return this.run(() -> {
            setTargetRPM(ShooterConstants.ALLIANCE_IDLE_RPM);
        }).finallyDo(interrupted -> stopShooting());
    }

    public Command setNeutralIdle() {
        return this.run(() -> {
            setTargetRPM(ShooterConstants.NEUTRAL_IDLE_RPM);
        }).finallyDo(interrupted -> stopShooting());
    }

    public Command setTargetRPMCommand(double rpm) {
        return this.run(() -> {
            setTargetRPM(rpm);
        }).finallyDo(interrupted -> stopShooting());
    }

    public Command stopShootingCommand() {
        return this.run(() -> {
            stopShooting();
        });
    }

    public Command shootingTestCommad() {
        return this.run(() -> {
            shootFuel();
        }).finallyDo(interrupted -> stopShooting());
    }



    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command idleCommand() {
    return this.run(() -> setTargetRPM(ShooterConstants.ALLIANCE_IDLE_RPM));
}

    @Override
    public void periodic() {
        isShooterRunning();

        double rightRPS = ShooterRightMotor.getVelocity().getValueAsDouble();
        double leftRPS = ShooterLeftMotor.getVelocity().getValueAsDouble();

        Logger.recordOutput("Shooter/RightRPM", RPSToRPM(rightRPS));
        Logger.recordOutput("Shooter/LeftRPM", RPSToRPM(leftRPS));
        Logger.recordOutput("Shooter/AverageRPM", RPSToRPM(getRPS()));
        Logger.recordOutput("Shooter/TargetRPM", RPSToRPM(targetRPS));
        Logger.recordOutput("Shooter/RightVoltage", ShooterRightMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/LeftVoltage", ShooterLeftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/RightCurrentDraw", ShooterRightMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/LeftCurrentDraw", ShooterLeftMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/IsShooterFast", isShooterFast());
    }
}