package frc.robot.utils;

import java.util.ArrayDeque;

public class ShotDetector {

    private final double dipRpm;
    private final double windowSeconds;
    private final double cooldownSeconds;
    private final ArrayDeque<double[]> recent = new ArrayDeque<>();
    private double lastShotSeconds = Double.NEGATIVE_INFINITY;
    private double releaseRpm = 0.0;
    private int count = 0;

    public ShotDetector(double dipRpm, double windowSeconds, double cooldownSeconds) {
        this.dipRpm = dipRpm;
        this.windowSeconds = windowSeconds;
        this.cooldownSeconds = cooldownSeconds;
    }

    public boolean update(double timeSeconds, double rpm, boolean feeding) {
        if (!feeding) {
            recent.clear();
            return false;
        }
        recent.addLast(new double[] {timeSeconds, rpm});
        while (recent.size() > 1 && timeSeconds - recent.peekFirst()[0] > windowSeconds) {
            recent.removeFirst();
        }
        double peak = rpm;
        for (double[] sample : recent) {
            peak = Math.max(peak, sample[1]);
        }
        if (peak - rpm >= dipRpm && timeSeconds - lastShotSeconds >= cooldownSeconds) {
            count++;
            lastShotSeconds = timeSeconds;
            releaseRpm = peak;
            recent.clear();
            recent.addLast(new double[] {timeSeconds, rpm});
            return true;
        }
        return false;
    }

    public int getCount() {
        return count;
    }

    public double getReleaseRpm() {
        return releaseRpm;
    }
}
