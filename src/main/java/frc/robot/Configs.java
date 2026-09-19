package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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
                        HoodMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);

                        HoodMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(HoodConstants.p)
                        .i(HoodConstants.i)
                        .d(HoodConstants.d)
                        .allowedClosedLoopError(HoodConstants.CLOSED_LOOP_DEADBAND_DEGREES / 360.0
                                * HoodConstants.GEAR_RATIO, ClosedLoopSlot.kSlot0)
                        .outputRange(-HoodConstants.MAX_OUTPUT, HoodConstants.MAX_OUTPUT)
                        .feedForward
                        .kS(HoodConstants.s)
                        .kV(HoodConstants.v)
                        .kA(HoodConstants.a)
;
                }
        }

        public static final class KickerSubsystem {

                public static final SparkMaxConfig VerticalMotorConfig = new SparkMaxConfig();

                static {
                        VerticalMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(false);

                        VerticalMotorConfig.closedLoop
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

                public static final SparkFlexConfig IntakeConfig = new SparkFlexConfig();

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
                        TurretMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12)
                        .inverted(true);

                        // REV Through Bore on the data port. Reported in encoder shaft degrees,
                        // zero centered so the range is (-180, 180] instead of [0, 360).
                        TurretMotorConfig.absoluteEncoder
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
