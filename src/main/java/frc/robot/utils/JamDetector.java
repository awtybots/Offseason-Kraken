package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

public class JamDetector {

    private final double jamCurrent;
    private final double stallSpeedFraction;
    private final double ignoreSeconds;
    private final double reverseSeconds;
    private final Debouncer debouncer;
    private final Timer feedTimer = new Timer();
    private final Timer reverseTimer = new Timer();
    private boolean feeding = false;

    public JamDetector(double jamCurrent, double ignoreSeconds, double debounceSeconds, double reverseSeconds) {
        this(jamCurrent, Double.POSITIVE_INFINITY, ignoreSeconds, debounceSeconds, reverseSeconds);
    }

    public JamDetector(double jamCurrent, double stallSpeedFraction, double ignoreSeconds, double debounceSeconds,
            double reverseSeconds) {
        this.jamCurrent = jamCurrent;
        this.stallSpeedFraction = stallSpeedFraction;
        this.ignoreSeconds = ignoreSeconds;
        this.reverseSeconds = reverseSeconds;
        this.debouncer = new Debouncer(debounceSeconds, DebounceType.kRising);
    }

    public boolean shouldReverse(double current) {
        return shouldReverse(current, 0.0);
    }

    public boolean shouldReverse(double current, double speedFraction) {
        if (!feeding) {
            feeding = true;
            feedTimer.restart();
        }
        if (reverseTimer.isRunning()) {
            if (!reverseTimer.hasElapsed(reverseSeconds)) {
                return true;
            }
            reverseTimer.stop();
            feedTimer.restart();
        }
        if (debouncer.calculate(feedTimer.hasElapsed(ignoreSeconds) && current > jamCurrent
                && speedFraction < stallSpeedFraction)) {
            reverseTimer.restart();
            return true;
        }
        return false;
    }

    public boolean isReversing() {
        return reverseTimer.isRunning();
    }

    public void reset() {
        feeding = false;
        reverseTimer.stop();
    }
}
