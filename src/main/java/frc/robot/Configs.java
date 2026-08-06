package frc.robot;

// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.TurretConstants;

public final class Configs 
{

        public static final class HoodSubsystem {

                public static final SparkMaxConfig HoodMotorConfig = new SparkMaxConfig();

                static {
                        HoodMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20).voltageCompensation(12);

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

        public static final class KickerSubsystem {

                public static final SparkMaxConfig VertivalMotorConfig = new SparkMaxConfig();

                static {
                        VertivalMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20).voltageCompensation(12);

                        VertivalMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(KickerConstants.VRp)
                        .i(KickerConstants.VRi)
                        .d(KickerConstants.VRd)
                        .feedForward
                        .kS(KickerConstants.VRs)
                        .kV(KickerConstants.VRv)
                        .kA(KickerConstants.VRa);
                }
        }

        public static final class TurretSubsystem {

                public static final SparkMaxConfig TurretMotorConfig = new SparkMaxConfig();

                static {
                        TurretMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

                        TurretMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(TurretConstants.p)
                        .i(TurretConstants.i)
                        .d(TurretConstants.d)
                        .outputRange(-TurretConstants.MAX_OUTPUT, TurretConstants.MAX_OUTPUT)
                        .feedForward
                        .kS(TurretConstants.s)
                        .kV(TurretConstants.v)
                        .kA(TurretConstants.a);
                }
        }
        
}
