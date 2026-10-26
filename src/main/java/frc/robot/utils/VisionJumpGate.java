package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.LimelightConstants;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;

public class VisionJumpGate {

    private final Map<String, Deque<Translation2d>> rejectedStreaks = new HashMap<>();
    private final Map<String, Double> lastTimestamps = new HashMap<>();

    public boolean isNewFrame(String camera, double timestampSeconds) {
        Double last = lastTimestamps.put(camera, timestampSeconds);
        return last == null || last != timestampSeconds;
    }

    public boolean accept(String camera, Pose2d estimate, int tagCount, Pose2d current) {
        Deque<Translation2d> streak = rejectedStreaks.computeIfAbsent(camera, k -> new ArrayDeque<>());
        double limit = tagCount >= 2
                ? LimelightConstants.MAX_JUMP_MULTI_TAG_M
                : LimelightConstants.MAX_JUMP_SINGLE_TAG_M;
        Translation2d seen = estimate.getTranslation();
        if (seen.getDistance(current.getTranslation()) <= limit) {
            streak.clear();
            return true;
        }
        streak.addLast(seen);
        while (streak.size() > LimelightConstants.RELOCALIZE_FRAMES) {
            streak.removeFirst();
        }
        if (streak.size() == LimelightConstants.RELOCALIZE_FRAMES
                && streak.stream().allMatch(p -> p.getDistance(seen) <= LimelightConstants.RELOCALIZE_AGREE_M)) {
            streak.clear();
            return true;
        }
        return false;
    }
}
