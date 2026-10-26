package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

public class StallHomer {

    public enum Step {
        IDLE, DRIVE, ZERO, GAVE_UP
    }

    private final double stallVelocity;
    private final double minSeconds;
    private final double settleSeconds;
    private final double timeoutSeconds;
    private final Timer timer = new Timer();
    private Debouncer stall;
    private boolean homed = false;
    private boolean homing = false;
    private boolean timedOut = false;

    public StallHomer(double stallVelocity, double minSeconds, double settleSeconds, double timeoutSeconds) {
        this.stallVelocity = stallVelocity;
        this.minSeconds = minSeconds;
        this.settleSeconds = settleSeconds;
        this.timeoutSeconds = timeoutSeconds;
    }

    public Step update(boolean enabled, double velocity) {
        if (homed) {
            return Step.IDLE;
        }
        if (!enabled) {
            homing = false;
            return Step.IDLE;
        }
        if (!homing) {
            homing = true;
            timer.restart();
            stall = new Debouncer(settleSeconds, DebounceType.kRising);
        }
        if (stall.calculate(timer.hasElapsed(minSeconds) && Math.abs(velocity) < stallVelocity)) {
            finish(false);
            return Step.ZERO;
        }
        if (timer.hasElapsed(timeoutSeconds)) {
            finish(true);
            return Step.GAVE_UP;
        }
        return Step.DRIVE;
    }

    private void finish(boolean gaveUp) {
        homed = true;
        homing = false;
        timedOut = gaveUp;
        timer.stop();
    }

    public void rehome() {
        homed = false;
        homing = false;
        timedOut = false;
    }

    public void markHomed() {
        homed = true;
        homing = false;
    }

    public boolean isHomed() {
        return homed;
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean timedOut() {
        return timedOut;
    }
}
