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
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust if needed
        rightConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.Slot0.kP = ShooterConstants.p;
        rightConfig.Slot0.kI = ShooterConstants.i;
        rightConfig.Slot0.kD = ShooterConstants.d;
        rightConfig.Slot0.kS = ShooterConstants.s;
        rightConfig.Slot0.kV = ShooterConstants.v;
        rightConfig.Slot0.kA = ShooterConstants.a;
        ShooterRightMotor.getConfigurator().apply(rightConfig);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ShooterLeftMotor.getConfigurator().apply(leftConfig);

        // Follow the right motor
        ShooterLeftMotor.setControl(new Follower(ShooterRightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean isShooterFast() {
        double avgRPS = (ShooterRightMotor.getVelocity().getValueAsDouble()
                + ShooterLeftMotor.getVelocity().getValueAsDouble()) / 2.0;
        return Math.abs(avgRPS - ShooterConstants.SHOOTER_SPEED) <= ShooterConstants.ERROR_MARGIN;
    }

    public double getRPS() {
        return (ShooterRightMotor.getVelocity().getValueAsDouble()
                + ShooterLeftMotor.getVelocity().getValueAsDouble()) / 2.0;
    }

    public boolean isShooterRunning() {
        double setpoint = Math.abs(targetRPS);
        boolean running = Math.abs(setpoint - getRPS()) < 1.67 // ~100 RPM in RPS
                && setpoint != 0
                && setpoint != ShooterConstants.ALLIANCE_IDLE_RPS;
        SmartDashboard.putBoolean("Shooter Running", running);
        return running;
    }

    public void shootFuel() {
        SpeedUpShooter();
    }

    public void stopShooting() {
        targetRPS = 0.0;
        ShooterRightMotor.setControl(dutyCycleRequest.withOutput(0.0));
        // ShooterLeftMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }

    public void SpeedUpShooter() {
        ShooterRightMotor.setControl(velocityRequest.withVelocity(ShooterConstants.SHOOTER_SPEED).withSlot(0));
        // ShooterLeftMotor.setControl(velocityRequest.withVelocity(ShooterConstants.SHOOTER_SPEED).withSlot(0));
    }

    public void setTargetRPS(double rps) {
        targetRPS = rps;
        ShooterRightMotor.setControl(velocityRequest.withVelocity(rps).withSlot(0));
        // ShooterLeftMotor.setControl(velocityRequest.withVelocity(rps).withSlot(0));
    }

    public void ShooterPassing() {
        ShooterRightMotor.setControl(velocityRequest.withVelocity(ShooterConstants.SHOOTER_PASSING_SPEED).withSlot(0));
        // ShooterLeftMotor.setControl(velocityRequest.withVelocity(ShooterConstants.SHOOTER_PASSING_SPEED).withSlot(0));
    }

    public Command setAllianceIdle() {
        return this.run(() -> {
            setTargetRPS(ShooterConstants.ALLIANCE_IDLE_RPS);
        }).finallyDo(interrupted -> stopShooting());
    }

    public Command setNeutralIdle() {
        return this.run(() -> {
            setTargetRPS(ShooterConstants.NEUTRAL_IDLE_RPS);
        }).finallyDo(interrupted -> stopShooting());
    }

    public Command setTargetRPSCommand(double rps) {
        return this.run(() -> {
            setTargetRPS(rps);
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

    @Override
    public void periodic() {
        isShooterRunning();

        double rightRPS = ShooterRightMotor.getVelocity().getValueAsDouble();
        double leftRPS = ShooterLeftMotor.getVelocity().getValueAsDouble();

        Logger.recordOutput("Shooter/RightRPS", rightRPS);
        Logger.recordOutput("Shooter/LeftRPS", leftRPS);
        Logger.recordOutput("Shooter/AverageRPS", getRPS());
        Logger.recordOutput("Shooter/TargetRPS", targetRPS);
        Logger.recordOutput("Shooter/RightAppliedVolts", ShooterRightMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/LeftAppliedVolts", ShooterLeftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/IsShooterFast", isShooterFast());
    }
}