package frc.robot;

// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.TurretConstants;

public final class Configs 
{

        public static final class HoodSubsystem {

                public static final SparkMaxConfig HoodMotorConfig = new SparkMaxConfig();

                static {
                        // kBrake, not kCoast: the hood has no absolute encoder and seeds its
                        // zero from HOOD_MIN_DEGREES at boot, so drift while disabled becomes a
                        // permanent offset - and at 60:1 that is 12x more motor rotations of
                        // error than it used to be. 20 A is REV's ceiling for a NEO 550; at 60:1
                        // the hood can stall against its own travel limits.
                        HoodMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

                        HoodMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(HoodConstants.p)
                        .i(HoodConstants.i)
                        .d(HoodConstants.d)
                        .outputRange(-HoodConstants.MAX_OUTPUT, HoodConstants.MAX_OUTPUT)
                        .feedForward
                        .kS(HoodConstants.s)
                        .kV(HoodConstants.v)
                        .kA(HoodConstants.a);
                }
        }

        public static final class KickerSubsystem {

                public static final SparkMaxConfig VertivalMotorConfig = new SparkMaxConfig();

                static {
                        VertivalMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(true);

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

        public static final class IntakeSubsystem {

                public static final SparkMaxConfig IntakeConfig = new SparkMaxConfig();

                static {
                        IntakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(true);

                        IntakeConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(IntakeConstants.p)
                        .i(IntakeConstants.i)
                        .d(IntakeConstants.d)
                        .feedForward
                        .kS(IntakeConstants.s)
                        .kV(IntakeConstants.v)
                        .kA(IntakeConstants.a);
                }
        }

        public static final class TurretSubsystem {

                public static final SparkMaxConfig TurretMotorConfig = new SparkMaxConfig();

                static {
                        TurretMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12)
                        .inverted(true);

                        // REV Through Bore on the data port. Reported in encoder shaft degrees,
                        // zero centered so the range is (-180, 180] instead of [0, 360).
                        TurretMotorConfig.absoluteEncoder
                        .setSparkMaxDataPortConfig()
                        .inverted(TurretConstants.ABSOLUTE_ENCODER_INVERTED)
                        .zeroOffset(TurretConstants.ABSOLUTE_ENCODER_OFFSET)
                        .zeroCentered(true)
                        .positionConversionFactor(360.0)
                        .velocityConversionFactor(360.0 / 60.0); // rpm -> deg per sec

                        // closed loop stays on the NEO's internal encoder: the through bore turns
                        // 10x per turret revolution so its reading alone is ambiguous
                        TurretMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(TurretConstants.p)
                        .i(TurretConstants.i)
                        .d(TurretConstants.d)
                        .outputRange(-TurretConstants.MAX_OUTPUT, TurretConstants.MAX_OUTPUT) //do 0.66 after testing
                        .feedForward
                        .kS(TurretConstants.s)
                        .kV(TurretConstants.v)
                        .kA(TurretConstants.a);
                }
        }
        
}
