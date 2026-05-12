// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Kraken extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX IshaanAndAdityasKraken = new TalonFX(1);
  VelocityVoltage setpoint = new VelocityVoltage(0);

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
  }


  public void runMotor()
  {
    IshaanAndAdityasKraken.setControl(setpoint.withVelocity(20).withAcceleration(50));
  }

  public void runMotorFast()
  {
    IshaanAndAdityasKraken.setControl(setpoint.withVelocity(50).withAcceleration(50));
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
        });
  }

  public Command runMotorFastCommand() {
    return this.run(
        () -> {
          runMotorFast();
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
