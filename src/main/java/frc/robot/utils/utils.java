package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

public class utils {
    public double RPMToRPS(double rpm)
    {
        return rpm / 60;
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
}
