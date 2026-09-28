package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

public class JamDetector {

    private final double jamCurrent;
    private final double ignoreSeconds;
    private final double reverseSeconds;
    private final Debouncer debouncer;
    private final Timer feedTimer = new Timer();
    private final Timer reverseTimer = new Timer();
    private boolean feeding = false;

    public JamDetector(double jamCurrent, double ignoreSeconds, double debounceSeconds, double reverseSeconds) {
        this.jamCurrent = jamCurrent;
        this.ignoreSeconds = ignoreSeconds;
        this.reverseSeconds = reverseSeconds;
        this.debouncer = new Debouncer(debounceSeconds, DebounceType.kRising);
    }

    public boolean shouldReverse(double current) {
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
        if (debouncer.calculate(feedTimer.hasElapsed(ignoreSeconds) && current > jamCurrent)) {
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
