package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private double desiredPercent = 0.0;

    private SparkFlex intakeMotor = new SparkFlex(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    private SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

    public Intake() {

        intakeMotor.configure(Configs.IntakeSubsystem.IntakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void runOuttake() {
        desiredPercent = IntakeConstants.OUTTAKE_DUTY;
        intakeController.setSetpoint(IntakeConstants.OUTTAKE_DUTY, ControlType.kDutyCycle);
    }

    public void runIntake() {
        desiredPercent = IntakeConstants.INTAKE_DUTY;
        intakeController.setSetpoint(IntakeConstants.INTAKE_DUTY, ControlType.kDutyCycle);
    }

    public void stopIntake() {
        desiredPercent = 0.0;
        intakeController.setSetpoint(0.0, ControlType.kDutyCycle);

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
        Logger.recordOutput("Intake/DesiredPercent", desiredPercent);
        Logger.recordOutput("Intake/Voltage", intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput());
        Logger.recordOutput("Intake/CurrentDraw", intakeMotor.getOutputCurrent() * intakeMotor.getAppliedOutput());
        Logger.recordOutput("Intake/Velocity", intakeController.getSetpoint());
        Logger.recordOutput("Intake/TargetVelocity", IntakeConstants.INTAKE_RPM);
    }
}