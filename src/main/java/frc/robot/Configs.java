package frc.robot;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
// import com.revrobotics.spark.ClosedLoopSlot;

public final class Configs 
{

        public static final class IntakeSubsystem {
                
            
            // public static final SparkFlexConfig IntakeRightMotorConfig = new SparkFlexConfig();
            private TalonFX IshaanAndAdityasKraken = new TalonFX(1);

                static {
                        TalonFXConfiguration IshaanAndAdityasKrakenConfig = new TalonFXConfiguration();

                        IshaanAndAdityasKrakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                        IshaanAndAdityasKrakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                        IshaanAndAdityasKrakenConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps
                        IshaanAndAdityasKrakenConfig.CurrentLimits.StatorCurrentLimitEnable = true;



                        




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
