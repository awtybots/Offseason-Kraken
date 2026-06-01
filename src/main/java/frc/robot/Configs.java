package frc.robot;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.TurretConstants;

public final class Configs 
{

            public static final class TurretSubsystem {

        public static final SparkMaxConfig TurretMotorConfig = new SparkMaxConfig();

        static {
            TurretMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

            TurretMotorConfig.encoder
                .positionConversionFactor(TurretConstants.RELATIVE_DEGREES_PER_ROTATION)
                .velocityConversionFactor(TurretConstants.RELATIVE_DEGREES_PER_ROTATION / 60.0);

            TurretMotorConfig.absoluteEncoder
                .positionConversionFactor(360.0)
                .velocityConversionFactor(360.0 / 60.0)
                .inverted(false); 

            TurretMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(TurretConstants.p)
                .i(TurretConstants.i)
                .d(TurretConstants.d)
                .outputRange(-TurretConstants.MAX_OUTPUT, TurretConstants.MAX_OUTPUT);
        }
    }

        
}
