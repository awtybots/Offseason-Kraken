package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ControlAllShooting;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.util.Color;

public class HubTrackerSubsystem extends SubsystemBase
{

    private Field2d field = new Field2d();
    private final SwerveSubsystem drivebase;
    private FieldObject2d circle;
    private FieldObject2d dynamicHubCircle;
    private FieldObject2d traj;

    final CommandXboxController driverController;

    boolean active = true;

    Color GREEN = Color.kGreen;
    Color RED = Color.kRed;

    private Pose2d hubPose;
    private double radius = 0.5; // radius to represent time left (circle gets smaller when shift ending)

    boolean show = false;
    
    int x = 0;
    
    public HubTrackerSubsystem(SwerveSubsystem drivebase, CommandXboxController driverController)
    {
        SmartDashboard.putString("Hub Color For Xavier", RED.toHexString());
        this.drivebase = drivebase;
        circle = field.getObject("Circle"); 
        traj = field.getObject("Trajectory");
        dynamicHubCircle = field.getObject("DynamicHubCircle");
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        hubPose = switch (alliance)
        {
            case Blue -> new Pose2d(4.6, 4, new Rotation2d());
            case Red -> new Pose2d(11.9, 4, new Rotation2d());
            default -> new Pose2d(4.6, 4.1, new Rotation2d());
        };
        SmartDashboard.putData("Field", field);
        this.driverController = driverController;
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty())
        {
          SmartDashboard.putNumber("TimeLeft", -1);
          return false;
        }
        if(!DriverStation.isFMSAttached())
        {
          SmartDashboard.putNumber("TimeLeft", -1);
          return false;
        }

        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomousEnabled()) // Always enabled in auton
        {
            int seconds = (20 - (int)matchTime);
            SmartDashboard.putNumber("TimeLeft", seconds);
            vibrate(seconds / 20.0);
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {SmartDashboard.putNumber("TimeLeft", -1);return false;} // If we're disabled then were probably not playing

        
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) return true;

        boolean isRedActiveFirst;
        switch (gameData.charAt(0)) {
        case 'R':
            isRedActiveFirst = false;
            break;
        case 'B':
            isRedActiveFirst = true;
            break;
        default:
            SmartDashboard.putNumber("TimeLeft", -1);
            return false;
        }

        boolean shiftOneActive = switch (alliance.get()) {
        case Red -> isRedActiveFirst;
        case Blue -> !isRedActiveFirst;
        };

        if (matchTime >= 130) // transition shift, always active
        {
            publish(140, (int)matchTime, 10.0);
            return true;
        } 
        else if (matchTime >= 105) { // shift 1
            publish(130, (int)matchTime, 25.0);
            return shiftOneActive;
        } 
        else if (matchTime >= 80) // shift 2
        {
            publish(105, (int)matchTime, 25.0);
            return !shiftOneActive;
        } 
        else if (matchTime >= 55) // shift 3
        {
            publish(80, (int)matchTime, 25.0);
            return shiftOneActive;
        } 
        else if (matchTime >= 30) // shift 4
        {
            publish(55, (int)matchTime, 25.0);
            return !shiftOneActive;
        } 
        else
        {
            int seconds = (int)matchTime;
            SmartDashboard.putNumber("TimeLeft", seconds);
            vibrate(seconds / 30.0);
            return true; // Endgame, always active
        }
  }

  private void publish(int time, int matchTime, double shiftTime)
  {
    double seconds = shiftTime -  (time - matchTime);
    SmartDashboard.putNumber("TimeLeft", seconds);
    if(seconds <= 5)
    {
      if(x == 1) SmartDashboard.putString("Hub Color For Xavier", GREEN.toHexString());
      else SmartDashboard.putString("Hub Color For Xavier", RED.toHexString());
    }
    else
    {
      if(active) SmartDashboard.putString("Hub Color For Xavier", GREEN.toHexString());
      else SmartDashboard.putString("Hub Color For Xavier", RED.toHexString());
    }
    radius = seconds / shiftTime;
    vibrate(radius);
  }

//   public Command isHubActiveCommand()
//   {
//     return run(()->
//     {
//         isHubActive();
//     });
//   }

  private void vibrate(double r)
  {
    if(r <= 0.15 && DriverStation.isFMSAttached() && DriverStation.isTeleop())
    {
        driverController.setRumble(RumbleType.kBothRumble, 0.15 * Math.pow((1.0 - r), 2));
    }
    else driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  public List<Pose2d> createCircle(Pose2d center, double r, int pts)
  {
    List<Pose2d> circle = new ArrayList<>();
    for(int i=0;i<pts;++i)
    {
        double angle = (i * (360.0/(double)pts)) * Math.PI / 180;
        
        double xOffset = r * Math.cos(angle);
        double yOffset = r * Math.sin(angle);

        circle.add(new Pose2d(center.getX() + xOffset, center.getY() + yOffset, new Rotation2d()));
    }


    return circle;
  }
  

  public void runPeriodic()
  {
    Pose2d robotPose = drivebase.getPose();

    field.setRobotPose(robotPose);
    active = isHubActive();

    x = (x == 1) ? 0 : 1;

    SmartDashboard.putBoolean("HubActivity", active);
    if(!active || x == 0) circle.setPoses(createCircle(hubPose, radius, 20));
    else circle.setPoses(); // clears circle when not showing

    dynamicHubCircle.setPoses(createCircle(drivebase.getCachedDynamicHubLocation(), 0.25, 10));

    // Translation2d goalLocation = drivebase.getCachedDynamicHubLocation().getTranslation();
    // Translation2d robotLocation = robotPose.getTranslation();
    // Translation2d targetVec = goalLocation.minus(robotLocation);
    // double        dist         = targetVec.getNorm();


    // Translation2d back = new Translation2d(-dist, 0);
    // Transform2d t = new Transform2d(back, new Rotation2d());

    // Pose2d start = robotPose;
    // Pose2d end = robotPose.plus(t);

    // Rotation2d angle = end.getTranslation().minus(start.getTranslation()).getAngle();
    
    // start = new Pose2d(start.getX(), start.getY(), angle);
    // end = new Pose2d(end.getX(), end.getY(), angle);

    // traj.setPoses(start, end);

    // SmartDashboard.putNumber("Distance to Hub", dist);
    // SmartDashboard.putNumber("LUTRPM", ControlAllShooting.getRPM(dist));
  }

  @Override
  public void periodic()
  {
    runPeriodic();
  }

  @Override
  public void simulationPeriodic()
  {
    runPeriodic();
  }
}