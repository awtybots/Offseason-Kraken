package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.RPM;

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

    public static final Translation2d TURRET_OFFSET = new Translation2d(
        (-29.5 / 2.0 + 3.0) * 0.0254,
        (24.5 / 2.0 - 3.0) * 0.0254);
    public static final double SHOOTER_OFFSET_FWD_M = TURRET_OFFSET.getX(); // -11.75" behind center
    public static final double SHOOTER_OFFSET_LEFT_M = TURRET_OFFSET.getY(); // 9.25" left of center

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

    // Boundary between the ALLIANCE ZONE and the NEUTRAL ZONE: the near face of
    // each HUB, not its center. Read off the official 2026-rebuilt-welded AprilTag
    // layout - tag 26 is the blue hub's west face, tag 10 the red hub's east face.
    // The old 182"/469" were the hub CENTERS, which claimed an extra 0.606 m band
    // on each side as "in zone" and flipped hub/ferry within a meter of the target.
    public static final double BLUE_ALLIANCE_ZONE_X_M = 4.0219;
    public static final double RED_ALLIANCE_ZONE_X_M = 12.5192;

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
    public static final int INTAKE_ID = 9; // unknown

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
    public static final int PUSHOUT_ID = 19; // set CAN ID

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
    public static final int SHOOTER_L_ID = 16;
    public static final int SHOOTER_R_ID = 17;

    public static final double SHOOTER_SPEED = 20;
    public static final double SHOOTER_PASSING_SPEED = 20;
    public static final double ERROR_MARGIN = 5; // RPS
    public static final double STOP = 0;
    public static final double IDLE = 0.1;

    public static final double ALLIANCE_IDLE_RPS = 30;
    public static final double ALLIANCE_AUTO_RPS = 30;
    public static final double NEUTRAL_IDLE_RPS = 0;

    // Phoenix 6 VelocityVoltage takes ROTATIONS PER SECOND and these gains are
    // volts per rps. The old values were volts per RPM - 60x too small - so the
    // loop asked for 0.16 V at a 4600 RPM setpoint and the flywheel never spun up.
    // Tell-tale: 0.00169 * 60 = 0.101, right next to the theoretical Kraken kV.
    //
    // kV is derived, not tuned: Kraken x60 free speed 6000 RPM = 100 rps at 12 V.
    // kP/kD are the old values carried across the same 60x, and still want a real
    // sysid - the routine is already wired up in Shooter.java.
    public static final double p = 0.0234;
    public static final double i = 0.000;
    public static final double d = 0.0; // was 0.0065; carrying it across the 60x would
                                        // give 0.39 V per rps/s, and derivative on a
                                        // noisy flywheel velocity signal just chatters.
                                        // 0 is the normal starting point for a flywheel.

    public static final double s = 0.0;
    public static final double v = 0.12; // 12 V / 100 rps
    public static final double a = 0.0;

    // ---- SHOOTER MECHANISM ----
    // Two Krakens both drive a common belt/pulley train. The rollers are locked to
    // each other at 3:2 - the bottom turns 3 for every 2 of the top - which is what
    // produces the backspin. The motors stay speed-matched (hence the Follower), and
    // one motor revolution is one BOTTOM roller revolution.
    //
    //   v_ball  = EFF * w_motor * (R_BOTTOM + PULLEY_TOP_PER_BOTTOM * R_TOP) / 2
    //   spin S  = (R_BOTTOM - PULLEY * R_TOP) / (R_BOTTOM + PULLEY * R_TOP)
    //
    // Going 1:1 -> 3:2 dropped ball speed per motor rev by 8.33% (so the RPM tables
    // rose 9.09%) and raised backspin per unit speed from S=0.500 to S=0.636, +27%.
    public static final double ROLLER_RADIUS_BOTTOM_M = 1.5 * 0.0254;
    public static final double ROLLER_RADIUS_TOP_M = 0.5 * 0.0254;
    public static final double PULLEY_TOP_PER_BOTTOM = 2.0 / 3.0;
    public static final double SHOOTER_EFFICIENCY = 0.90; // grip/slip loss into the ball

    // ---- AERO (parameters the tables below were generated with) ----
    // Linear drag time constant. Kept at 0.45 rather than 6328's 0.375: their value
    // is for a ball with far less spin, and a sphere at S=0.64 carries noticeably
    // more drag than the spin-free C_d=0.63 that 5987 measured by dropping it.
    public static final double LINEAR_DRAG_K = 0.45;
    // Magnus lift as a fraction of drag. Both scale with v^2 about the same way, so
    // k_magnus = LINEAR_DRAG_K * MAGNUS_LIFT_RATIO and this is just C_L / C_D.
    // C_L ~ 0.30 at S = 0.64 (spheres saturate near 0.3-0.4). TOF is sensitive to
    // this (+-7% over 0.30-0.60); required RPM is not (under 1%).
    public static final double MAGNUS_LIFT_RATIO = 0.45;

    public final static InterpolatingDoubleTreeMap TOF = new InterpolatingDoubleTreeMap();

    // Flight time of the hubHoodTable shot, solved with linear drag AND Magnus lift
    // (see MECHANISM/AERO block above). Domain 2-6 m; InterpolatingDoubleTreeMap
    // clamps outside it, which is why ferry must NOT use this map - see the ferry
    // tables below.
    //
    // Magnus is what moves these numbers: it changes required launch speed by under
    // 1% but adds ~11% to flight time, because the lift vector is velocity rotated
    // 90 deg and points up-and-backward while the ball is still climbing.
    //
    // MODEL OUTPUT, NOT MEASURED. Still sits ~20% under every team that measured
    // this game piece; the residual is most likely arc choice, not aero.
    static {
      for (var entry : List.of(
          Pair.of(Meters.of(2.0), Seconds.of(0.809)),
          Pair.of(Meters.of(2.5), Seconds.of(0.881)),
          Pair.of(Meters.of(3.0), Seconds.of(0.954)),
          Pair.of(Meters.of(3.5), Seconds.of(1.025)),
          Pair.of(Meters.of(4.0), Seconds.of(1.094)),
          Pair.of(Meters.of(4.5), Seconds.of(1.160)),
          Pair.of(Meters.of(5.0), Seconds.of(1.228)),
          Pair.of(Meters.of(5.5), Seconds.of(1.291)),
          Pair.of(Meters.of(6.0), Seconds.of(1.355)))) {
        TOF.put(entry.getFirst().in(Meters), entry.getSecond().in(Seconds));
      }
    }

    public static final double TOF_SCALE = 1.0;   // tune up toward ~1.25 with real shots
    
    public static final InterpolatingDoubleTreeMap hubShooterTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap ferryShooterTable = new InterpolatingDoubleTreeMap();
    static {

      // BOTTOM roller RPM (= motor RPM). Solved for the launch speed that reaches
      // dz = 1.2954 m at each distance on the hubHoodTable angle, integrating linear
      // drag plus Magnus lift. Ball speed from the MECHANISM block above.
      //
      // Nearly all of the rise over the old 1:1 table is the pulley change, not aero:
      // Magnus is worth under 1% here. 4604 RPM at 6 m is 77% of Kraken x60 free
      // speed, so expect the far end to droop under load - if long shots land low
      // while short ones are fine, that is the flywheel running out, not the table.
      for (var entry : List.of(
          Pair.of(Meters.of(2.0), RPM.of(3054)),
          Pair.of(Meters.of(2.5), RPM.of(3251)),
          Pair.of(Meters.of(3.0), RPM.of(3452)),
          Pair.of(Meters.of(3.5), RPM.of(3651)),
          Pair.of(Meters.of(4.0), RPM.of(3848)),
          Pair.of(Meters.of(4.5), RPM.of(4040)),
          Pair.of(Meters.of(5.0), RPM.of(4232)),
          Pair.of(Meters.of(5.5), RPM.of(4418)),
          Pair.of(Meters.of(6.0), RPM.of(4604)))) {
        hubShooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM) / 60.0); // convert to RPS
      }

      // STILL PLACEHOLDERS - these were never derived from a trajectory, so all that
      // has been applied is the mechanical rescale: the 3:2 pulleys mean the same
      // ball speed needs 9.09% more motor RPM. The angles they pair with
      // (ferryHoodTable) are also invented and stop at 6 m while this runs to 10 m.
      // 5291 RPM at 10 m is 88% of Kraken free speed and will not hold under load.
      for (var entry : List.of(
          Pair.of(Meters.of(2.0), RPM.of(3047)),
          Pair.of(Meters.of(2.5), RPM.of(3253)),
          Pair.of(Meters.of(3.0), RPM.of(3459)),
          Pair.of(Meters.of(3.5), RPM.of(3663)),
          Pair.of(Meters.of(4.0), RPM.of(3864)),
          Pair.of(Meters.of(4.5), RPM.of(4063)),
          Pair.of(Meters.of(5.0), RPM.of(4257)),
          Pair.of(Meters.of(5.5), RPM.of(4447)),
          Pair.of(Meters.of(6.0), RPM.of(4634)),
          Pair.of(Meters.of(7.0), RPM.of(4800)),
          Pair.of(Meters.of(8.0), RPM.of(4964)),
          Pair.of(Meters.of(9.0), RPM.of(5127)),
          Pair.of(Meters.of(10.0), RPM.of(5291)))) {
        ferryShooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM) / 60.0); // convert to RPS
      }
    }

  }

  public static final class TurretConstants {
    public static final int TURRET_ID = 15; // set ts
    // 50:1 motor -> turret. Independent of ABSOLUTE_ENCODER_RATIO below: that one is
    // the through-bore's own gearing. Turret/FrameDisagreementDeg reads near zero
    // when each is individually correct, not because they match.
    public static final double GEAR_RATIO = 50.0;

    // REV Through Bore in the SPARK MAX data port (absolute encoder adapter)
    public static final double ABSOLUTE_ENCODER_RATIO = 10.0; // encoder revolutions per one full turret revolution
    public static final double ABSOLUTE_ENCODER_OFFSET = 0.0;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = false; // flip if the encoder counts down when the turret
                                                                   // goes counterclockwise

    public static final double REFERENCE_TURRET_DEGREES = 0.0; // zeroed facing straight forward towards the intake

    // how much they can spin each way (shouldnt be the same just is as a
    // placeholder for now)
    public static final double MIN_CONTINUOUS_DEGREES = -160.0; // TODO measure: how far CW it goes from forward
    public static final double MAX_CONTINUOUS_DEGREES = 160.0; // TODO measure: how far CCW it goes from forward

    // Keep this much air between the commanded setpoint and the hard stop. Clamping
    // straight to MIN/MAX parks the turret on the stop and leaves the position loop
    // pushing into it forever.
    public static final double CABLE_LIMIT_MARGIN_DEGREES = 2.0;

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
    public static final int HOOD_ID = 18; // set ts

    public static final double HOOD_MIN_DEGREES = 21.0; // down pos (starting pos)
    public static final double HOOD_MAX_DEGREES = 47.0; // up position

    // 60:1 motor -> hood. At the old 5:1 the NEO's 42-count encoder gave only 0.58
    // counts per hood degree, so ANGLE_TOLERANCE_DEGREES = 0.5 was finer than the
    // sensor could resolve and isAtAngle() was effectively noise. At 60:1 it is
    // 7.0 counts/degree, so half a degree is a real measurement.
    public static final double GEAR_RATIO = 60.0;
    public static final double ANGLE_TOLERANCE_DEGREES = 0.5;

    // The TRENCH sits at the HUB's x, so these double as the trench x band.
    public static final double TRENCH_X_BLUE = 4.6; // blue side trench x coordinate
    public static final double TRENCH_X_RED = 11.9; // red side trench x coordinate
    public static final double TRENCH_THRESHOLD = 0.6; // tuck when within this many meters of the trench (prolly needs
                                                       // to be lower)

    // The trench is NOT a band across the field - it is two openings along the
    // guardrails with the HUB sitting in the gap between them. Derived from the
    // 2026 field: width 8.069 m, hub 47" wide, bumps 73" wide, plus 12" of approach.
    // Without this y test the hood tucks itself in front of the hub, which is where
    // most shots are taken from.
    public static final double TRENCH_Y_RIGHT_MAX = 1.279; // opening on the y=0 guardrail
    public static final double TRENCH_Y_LEFT_MIN = 6.790; // opening on the y=8.069 guardrail

    // PID — tune on robot
    public static final double p = 0.1;
    public static final double i = 0.0;
    public static final double d = 0.0;
    public static final double MAX_OUTPUT = 0.5; // limit speed for safety while tuning

    public static final InterpolatingDoubleTreeMap hubHoodTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap ferryHoodTable = new InterpolatingDoubleTreeMap();

    static {
      // aim at hub LUT
      // Hood angle = 90 - ball_exit_angle. Exit angle chosen as the min-launch-speed
      // angle: theta_opt = 45 + 0.5 * atan(dz/d), with dz = 1.296 m (72" hub - 21"
      // shooter).
      for (var entry : List.of(
          Pair.of(Meters.of(2.0), Degrees.of(28.5)),
          Pair.of(Meters.of(2.5), Degrees.of(31.3)),
          Pair.of(Meters.of(3.0), Degrees.of(33.3)),
          Pair.of(Meters.of(3.5), Degrees.of(34.8)),
          Pair.of(Meters.of(4.0), Degrees.of(36.0)),
          Pair.of(Meters.of(4.5), Degrees.of(37.0)),
          Pair.of(Meters.of(5.0), Degrees.of(37.7)),
          Pair.of(Meters.of(5.5), Degrees.of(38.4)),
          Pair.of(Meters.of(6.0), Degrees.of(38.9)))) {
        hubHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
      }

      // aim at ferry LUT
      for (var entry : List.of(
          Pair.of(Meters.of(2.0), Degrees.of(20.0)),
          Pair.of(Meters.of(3.0), Degrees.of(25.0)),
          Pair.of(Meters.of(4.0), Degrees.of(30.0)),
          Pair.of(Meters.of(5.0), Degrees.of(35.0)),
          Pair.of(Meters.of(6.0), Degrees.of(40.0)))) {
        ferryHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
      }
    }
  }

  public static class KickerConstants {
    public static final int KICKER_ID = 14;
    public static final int VERT_ROLLER_ID = 11;

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

    // PID Constants
    public static final double VRp = 0.000236;
    public static final double VRi = 0.000;
    public static final double VRd = 0.000;

    // Feed-Forward Constants
    public static final double VRs = 0.100;
    public static final double VRv = 0.004;
    public static final double VRa = 0.0003;

    public static final double STOP = 0;
    public static final double IDLE = 0; // % voltage -1 --> 1
  }

  public static class ConveyorConstants {
    public static final int CONVEYOR_TOP_ID = 12;
    public static final int CONVEYOR_BOTTOM_ID = 13;

    public static final double CONVEYOR_REVERSE_SPEED = -0.85;
    public static final double CONVEYOR_SPEED = 0.85;

    public static final double CONVEYOR_RPS = 75;
    public static final double CONVEYOR_REVERSE_RPS = -75;

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
    public static final int ROLLERS_ID = 10;

    public static final double ROLLERS_RPS = -100;
    public static final double REVERSE_ROLLERS_RPS = 100;

    public static final double ROLLERS_SPEED = -1;
    public static final double REVERSE_ROLLERS_SPEED = 1;

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