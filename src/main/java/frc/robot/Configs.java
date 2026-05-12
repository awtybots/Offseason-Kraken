package frc.robot;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
// import com.revrobotics.spark.ClosedLoopSlot;

public final class Configs 
{

        public static final class IntakeSubsystem {
                
            
            // public static final SparkFlexConfig IntakeRightMotorConfig = new SparkFlexConfig();
            public static final TalonFXConfiguration IshaanAndAdityasKrakenConfig = new TalonFXConfiguration();

                static {

                        // IshaanAndAdityasKrakenConfig.withCurrentLimits(40).withVoltage(12);
                        // IntakeMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        // IntakeMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).inverted(true);



                        // IntakeMotorLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //     // Set PID values for position control. We don't need to pass a closed
                        //     // loop slot, as it will default to slot 0.
                        //     .p(IntakeConstants.p)
                        //     .i(IntakeConstants.i)
                        //     .d(IntakeConstants.d)
                        //     .outputRange(-1, 1)
                        //     .feedForward
                        //     .kS(IntakeConstants.s)
                        //     .kV(IntakeConstants.v)
                        //     .kA(IntakeConstants.a)
                        //     ;

                        // IntakeMotorLeftConfig.closedLoop
                        // .maxMotion.maxAcceleration(1000000);


                        // IntakeMotorRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //     // Set PID values for position control. We don't need to pass a closed
                        //     // loop slot, as it will default to slot 0.
                        //     .p(IntakeConstants.p)
                        //     .i(IntakeConstants.i)
                        //     .d(IntakeConstants.d)
                        //     .outputRange(-1, 1)
                        //     .feedForward
                        //     .kS(IntakeConstants.s)
                        //     .kV(IntakeConstants.v)
                        //     .kA(IntakeConstants.a)
                        //     ;

                        // IntakeMotorRightConfig.closedLoop
                        // .maxMotion.maxAcceleration(1000000);

                }

        };
        
}
