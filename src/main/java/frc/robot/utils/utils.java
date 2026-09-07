package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import org.littletonrobotics.junction.Logger;

public class utils {
    public static double RPMToRPS(double rpm)
    {
        return rpm / 60;
    }

    public static double RPSToRPM(double rps)
    {
        return rps * 60;
    }

    /**
     * Logs whether FOC is actually running on a TalonFX.
     *
     * EnableFOC is only a *request* -- an unlicensed device silently falls back to
     * trapezoidal commutation. getControlMode() reports what the motor is really
     * doing, and the FOC modes are the ones whose enum name ends in "FOC".
     */
    public static void logFOC(String key, TalonFX motor) {
        String controlMode = motor.getControlMode().getValue().toString();
        Logger.recordOutput(key + "/ControlMode", controlMode);
        Logger.recordOutput(key + "/FOCActive", controlMode.endsWith("FOC"));
        Logger.recordOutput(key + "/ProLicensed", motor.getIsProLicensed().getValue());
    }

    public static double getStatorCurrent(SparkFlex motor)
    {
        return motor.getOutputCurrent();
    }

    public static double getStatorCurrent(SparkMax motor)
    {
        return motor.getOutputCurrent();
    }

    public static double getStatorCurrent(TalonFX motor)
    {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public static double getAppliedVoltage(SparkFlex motor)
    {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    public static double getAppliedVoltage(SparkMax motor)
    {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    public static double getAppliedVoltage(TalonFX motor)
    {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public static double getBatteryVoltage(SparkFlex motor)
    {
        return motor.getBusVoltage();
    }

    public static double getBatteryVoltage(SparkMax motor)
    {
        return motor.getBusVoltage();
    }

    public static double getBatteryVoltage(TalonFX motor)
    {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    public static double getSupplyCurrent(SparkFlex motor)
    {
        return (getStatorCurrent(motor) * getAppliedVoltage(motor)) / getBatteryVoltage(motor);
    }

    public static double getSupplyCurrent(SparkMax motor)
    {
        return (getStatorCurrent(motor) * getAppliedVoltage(motor)) / getBatteryVoltage(motor);
    }

    public static double getSupplyCurrent(TalonFX motor)
    {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public static double getDutyCycle(TalonFX motor)
    {
        return motor.getDutyCycle().getValueAsDouble();
    }
}
