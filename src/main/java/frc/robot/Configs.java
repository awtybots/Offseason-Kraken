package frc.robot;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.KickerConstants;

public final class Configs 
{

        public static final class HoodSubsystem {

                public static final SparkMaxConfig HoodMotorConfig = new SparkMaxConfig();

                static {
                        HoodMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

                        HoodMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(HoodConstants.p)
                        .i(HoodConstants.i)
                        .d(HoodConstants.d)
                        .outputRange(-HoodConstants.MAX_OUTPUT, HoodConstants.MAX_OUTPUT)
                        .feedForward
                        .kS(KickerConstants.s)
                        .kV(KickerConstants.v)
                        .kA(KickerConstants.a);
                }
        }
        
}
