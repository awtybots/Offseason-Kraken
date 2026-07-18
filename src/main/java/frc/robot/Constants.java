package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean SIM_REPLAY_MODE = false;
  
  public static final double MAX_SPEED = Units.feetToMeters(16.5);


  public static final class DrivebaseConstants {

    public static final Pose3d redHubPose = new Pose3d(Units.inchesToMeters(469.09488), Units.inchesToMeters(158.6614),
        Units.inchesToMeters(72.0), new Rotation3d());
    public static final Pose3d blueHubPose = new Pose3d(Units.inchesToMeters(182.12598), Units.inchesToMeters(158.6614),
        Units.inchesToMeters(72.0), new Rotation3d());

    // shooter position: back left corner, 3" in from each edge, 21" above ground
    public static final double SHOOTER_HEIGHT_M = Units.inchesToMeters(21.0);
    public static final double SHOOTER_OFFSET_FWD_M = Units.inchesToMeters(-(29.5 / 2.0 - 3.0)); // -11.75" behind center
    public static final double SHOOTER_OFFSET_LEFT_M = Units.inchesToMeters(24.5 / 2.0 - 3.0);   // 9.25" left of center    

    public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    public static final Pose2d LT_ENTER_POS = new Pose2d(5.848, 7.241, Rotation2d.fromDegrees(90));
    public static final Pose2d RT_ENTER_POS = new Pose2d(5.839, 0.823, Rotation2d.fromDegrees(-90));

    public static final Pose2d getHubPose2D() {
      Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
      return pose.toPose2d();
    }

    public static final Pose3d getHubPose3D() {
      Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
      return pose;
    }
    

    public static final Pose2d getFerryPose(Translation2d robotPose) {
      if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
        if (robotPose.getDistance(redFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose
            .getDistance(redFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return redFerryPoseOutpost.toPose2d();
        } else {
          return redFerryPoseDepot.toPose2d();
        }
      } else {
        if (robotPose.getDistance(blueFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose
            .getDistance(blueFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return blueFerryPoseOutpost.toPose2d();
        } else {
          return blueFerryPoseDepot.toPose2d();
        }
      }
    }

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class LimelightConstants {
    public static final String LIMELIGHT_FRONT = "limelight-front";
    public static final String LIMELIGHT_BACK = "limelight-back";
    public static final String LIMELIGHT_LEFT = "limelight-left";
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // Port
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

  }

  public static class IntakeConstants {
    public static final int INTAKE_LEFT_ID = 18; // unknown
    public static final int INTAKE_RIGHT_ID = 19; // unknown

    // PID Constants
    public static final double p = 0.006155;
    public static final double i = 0.000;
    public static final double d = 0.01;

    // Feed-Forward Constants
    public static final double s = 1.25;
    public static final double v = 0.5;
    public static final double a = 0.75;

    public static final double OUTTAKE_SPEED = -1;
    public static final double INTAKE_SPEED = 1;
    public static final double INTAKE_RPS = 220;
    public static final double OUTTAKE_RPS = -220;

  }

  public static final class PushoutConstants {
      public static final int PUSHOUT_ID = 0; // set CAN ID

      // Positions in rotations (tune these to match your old encoder values)
      public static final double PUSHOUT_EXTENDED_POS = 15.0;
      public static final double PUSHOUT_RETRACTED_POS = 5.0;
      public static final double FULLY_RETRACTED_POS = 0.0;

      // PID/FF
      public static final double p = 1.0;
      public static final double i = 0.0;
      public static final double d = 0.0;
      public static final double s = 0.1;
      public static final double v = 0.12;
      public static final double a = 0.0;

      public static final double PUSHOUT_AGITATE_WAIT = 0.2; // seconds
      public static final double PUSHOUT_BETWEEN = 0.5; // seconds between in and out
  }

  public static class ShooterConstants {
    public static final int SHOOTER_R_ID = 9;
    public static final int SHOOTER_L_ID = 10;

    public static final double SHOOTER_SPEED = -100;                
    public static final double SHOOTER_PASSING_SPEED = -200;  
    public static final double ERROR_MARGIN = 5; // RPS         
    public static final double STOP = 0;
    public static final double IDLE = 0.1; 

    public static final double ALLIANCE_IDLE_RPS = -50;
    public static final double ALLIANCE_AUTO_RPS = -50;
    public static final double NEUTRAL_IDLE_RPS = 0;

    // PID Constants For Shooter
    public static final double p = 0.00039;
    public static final double i = 0.000;
    public static final double d = 0.0065;

    // Feed-Forward Constants for Shooter
    public static final double s = 0.0;
    public static final double v = 0.00169;
    public static final double a = 0.0;


    public final static InterpolatingDoubleTreeMap TOF = new InterpolatingDoubleTreeMap();

    static {
      for (var entry : List.of(
          Pair.of(Meters.of(2), Seconds.of(0.85)),
          Pair.of(Meters.of(3), Seconds.of(0.95)),
          Pair.of(Meters.of(4), Seconds.of(1.13)),
          Pair.of(Meters.of(5), Seconds.of(1.31)),
          Pair.of(Meters.of(6), Seconds.of(1.49)),
          Pair.of(Meters.of(7), Seconds.of(1.67)),
          Pair.of(Meters.of(8), Seconds.of(1.85)),
          Pair.of(Meters.of(9), Seconds.of(2.03)),
          Pair.of(Meters.of(10), Seconds.of(2.21)),
          Pair.of(Meters.of(11), Seconds.of(2.39)),
          Pair.of(Meters.of(12), Seconds.of(2.57)))) {
        TOF.put(entry.getFirst().in(Meters), entry.getSecond().in(Seconds));
      }
    }
  }

  public static final class TurretConstants {
    public static final int TURRET_ID = 0; // set ts
    public static final int TURRET_CANCODER_ID = 0;  // set ts too

    public static final double CANCODER_OFFSET = 0.0; // tune so 0 is the center of turret range
    public static final double GEAR_RATIO = 5.0; // tune to correct reduction

    public static final double RELATIVE_DEGREES_PER_ROTATION = 50; // set it to however many rotations of the motor it takes to spin the turret 360 degs

    public static final double MIN_CONTINUOUS_DEGREES = -160.0; // set this to the lower end of the turret's range for example: -160
    public static final double MAX_CONTINUOUS_DEGREES = 160.0;// higher end for example: 160 meaning total range is 320 degrees

    public static final double p = 0.05;
    public static final double i = 0.0;
    public static final double d = 0.0;
      
    public static final double s = 0.100;
    public static final double v = 0.004;
    public static final double a = 0.0003;

    public static final double ANGLE_TOLERANCE_DEGREES = 0.5;

    public static final double MAX_OUTPUT = 0.25; // speed limit to keep it safe for tuning
  }

  public static final class HoodConstants {
    public static final int HOOD_ID = 0; // set ts

    public static final double HOOD_MIN_DEGREES = 21.0;  // down pos (starting pos)
    public static final double HOOD_MAX_DEGREES = 47.0;  // up position

    public static final double GEAR_RATIO = 5.0;       
    public static final double ANGLE_TOLERANCE_DEGREES = 0.5; 

    public static final double TRENCH_X_BLUE = 4.6;      // blue side trench x coordinate
    public static final double TRENCH_X_RED = 11.9;       // red side trench x coordinate
    public static final double TRENCH_THRESHOLD = 0.6;    // tuck when within this many meters of the trench (prolly needs to be lower)

    // PID — tune on robot
    public static final double p = 0.1;
    public static final double i = 0.0;
    public static final double d = 0.0;
    public static final double MAX_OUTPUT = 0.5; // limit speed for safety while tuning
  }

  public static class KickerConstants {
    public static final int KICKER_ID = 13;
    public static final int VERT_ROLLER_ID = 14;

    public static final double KICKER_REVERSE_SPEED = -0.85; // adjust speeds
    public static final double KICKER_SPEED = 0.85;

    public static final double VERT_ROLLER_REVERSE_SPEED = -0.85;
    public static final double VERT_ROLLER_SPEED = 0.85;
    // PID Constants
    public static final double p = 0.000236;
    public static final double i = 0.000;
    public static final double d = 0.000;

    // Feed-Forward Constants
    public static final double s = 0.100;
    public static final double v = 0.004;
    public static final double a = 0.0003;

    public static final double STOP = 0;
    public static final double IDLE = 0; // % voltage -1 --> 1
  }

  public static class ConveyorConstants {
    public static final int CONVEYOR_TOP_ID = 13;
    public static final int CONVEYOR_BOTTOM_ID = 14;

    public static final double CONVEYOR_REVERSE_SPEED = -0.85;
    public static final double CONVEYOR_SPEED = 0.85;

    // PID Constants
    public static final double p = 0.000236;
    public static final double i = 0.000;
    public static final double d = 0.000;

    // Feed-Forward Constants
    public static final double s = 0.100;
    public static final double v = 0.004;
    public static final double a = 0.0003;

    public static final double STOP = 0;
    public static final double IDLE = 0; // % voltage -1 --> 1
  }


  public static class RollersConstants {
    public static final int ROLLERS_ID = 15;

    public static final double ROLLERS_RPS = -270;
    public static final double REVERSE_ROLLERS_RPS = 270;

    // PID Constants
    public static final double p = 0.0002;
    public static final double i = 0.000;
    public static final double d = 0.000;

    // Feed-Forward Constants
    public static final double s = 0.100;
    public static final double v = 0.00177;
    public static final double a = 0.00017;

  }

  // Object Detection
  public static final double X_FUEL_SETPOINT = 0.5;
  public static final double Y_FUEL_SETPOINT = 0.0;

  public static final double X_FUEL_TOLERANCE = 0.1;
  public static final double Y_FUEL_TOLERANCE = 0.1;


}