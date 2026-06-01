// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Kraken extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX IshaanAndAdityasKraken = new TalonFX(1);
  private TalonFX david = new TalonFX(2);

  VelocityVoltage setpoint = new VelocityVoltage(0);
  PositionVoltage position = new PositionVoltage(0);

  Orchestra orchestra = new Orchestra();
  Orchestra orchestra2 = new Orchestra();

  public Kraken() {
    TalonFXConfiguration IshaanAndAdityasKrakenConfig = new TalonFXConfiguration();

        IshaanAndAdityasKrakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        IshaanAndAdityasKrakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        IshaanAndAdityasKrakenConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps
        IshaanAndAdityasKrakenConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        IshaanAndAdityasKrakenConfig.Slot0.kP = 0.1;
        IshaanAndAdityasKrakenConfig.Slot0.kI = 0.0;
        IshaanAndAdityasKrakenConfig.Slot0.kD = 0.0;

        IshaanAndAdityasKrakenConfig.Slot0.kS = 0.1;
        IshaanAndAdityasKrakenConfig.Slot0.kV = 0.1;
        IshaanAndAdityasKrakenConfig.Slot0.kA = 0.0;

        IshaanAndAdityasKraken.getConfigurator().apply(IshaanAndAdityasKrakenConfig);

    TalonFXConfiguration DAVIDConfig = new TalonFXConfiguration();

        DAVIDConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        DAVIDConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        DAVIDConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps
        DAVIDConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        DAVIDConfig.Slot0.kP = 0.1;
        DAVIDConfig.Slot0.kI = 0.0;
        DAVIDConfig.Slot0.kD = 0.0;

        DAVIDConfig.Slot0.kS = 0.1;
        DAVIDConfig.Slot0.kV = 0.1;
        DAVIDConfig.Slot0.kA = 0.0;

        david.getConfigurator().apply(DAVIDConfig);         

  }


  public void runAway()
  {
      orchestra.play();
      orchestra2.play();
  }

  public void stopRunAway()
  {
    orchestra.stop();
    orchestra2.stop();
  }

  public void runMotor()
  {
    IshaanAndAdityasKraken.setControl(setpoint.withVelocity(20).withAcceleration(50).withSlot(0));

  }

  public void runMotorFast()
  {
    IshaanAndAdityasKraken.setControl(setpoint.withVelocity(50).withAcceleration(50).withSlot(0));
  }

  public void extendPosControl()
  {
    david.setControl(position.withPosition(100).withSlot(0));
  }

  public void retractPosControl()
  {
    david.setControl(position.withPosition(0).withSlot(0));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command runMotorCommand() {
    return this.run(
        () -> {
          runMotor();
        }).finallyDo(
        () -> {
          IshaanAndAdityasKraken.setControl(setpoint.withVelocity(0));
        });
  }

  public Command runMotorFastCommand() {
    return this.run(
        () -> {
          runMotorFast();
        }).finallyDo(
        () -> {
          IshaanAndAdityasKraken.setControl(setpoint.withVelocity(0));
        });
  }

  public Command extendCommand() {
    return this.run(
        () -> {
          extendPosControl();
        }).finallyDo(
        () -> {
          david.setControl(position.withPosition(100));
        });
  }

  public Command retractCommand() {
    return this.run(
        () -> {
          retractPosControl();
        }).finallyDo(
        () -> {
          david.setControl(position.withPosition(0));
        });
  }


  public Command runAwayCommand() {
    return this.run(
        () -> {
          runAway();
        });
  }

  public Command stopRunaway() {
    return this.run(
        () -> {
          stopRunAway();
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
