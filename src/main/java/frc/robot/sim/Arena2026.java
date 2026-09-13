package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * The 2026 REBUILT field as a maple-sim arena.
 *
 * <p>maple-sim ships arenas for reefscape2025 and an evergreen one that is only the perimeter,
 * so it knows the walls but nothing that stands on this year's field. That gap is the whole
 * reason a second collision system existed here. Registering the hub, tower and trench blocks
 * as real obstacles lets dyn4j resolve every contact - walls and structures alike - with one
 * solver, which is what makes a robot slide along a face and spin off a corner without any of
 * that behaviour being hand-written.
 *
 * <p>Install with {@link SimulatedArena#overrideInstance} BEFORE the {@code SwerveDrive} is
 * built: YAGSL registers its drivetrain with whatever instance exists at construction time.
 */
public class Arena2026 extends SimulatedArena {

  public Arena2026() {
    super(new Map2026());
  }

  /** FuelSim owns the game pieces, so the arena places none of its own. */
  @Override
  public void placeGamePiecesOnField() {}

  private static final class Map2026 extends FieldMap {

    private Map2026() {
      double length = SimRobot.SimConstants.FIELD_LENGTH_M;
      double width = SimRobot.SimConstants.FIELD_WIDTH_M;

      addBorderLine(new Translation2d(0, 0), new Translation2d(length, 0));
      addBorderLine(new Translation2d(length, 0), new Translation2d(length, width));
      addBorderLine(new Translation2d(length, width), new Translation2d(0, width));
      addBorderLine(new Translation2d(0, width), new Translation2d(0, 0));

      for (double[] box : SimRobot.OBSTACLES) {
        addRectangularObstacle(box[1] - box[0], box[3] - box[2],
            new Pose2d((box[0] + box[1]) / 2.0, (box[2] + box[3]) / 2.0, Rotation2d.kZero));
      }
    }
  }
}
