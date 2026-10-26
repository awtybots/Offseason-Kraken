package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

public class ControllerRumble {

    private final GenericHID hid;
    private final Timer pulseTimer = new Timer();
    private double pulseLevel = 0.0;
    private double pulseSeconds = 0.0;
    private double heldLevel = 0.0;
    private double appliedLevel = -1.0;

    public ControllerRumble(GenericHID hid) {
        this.hid = hid;
    }

    public void hold(double level) {
        heldLevel = Math.max(heldLevel, level);
    }

    public void pulse(double level, double seconds) {
        pulseLevel = level;
        pulseSeconds = seconds;
        pulseTimer.restart();
    }

    public void update() {
        double level = heldLevel;
        if (pulseTimer.isRunning() && !pulseTimer.hasElapsed(pulseSeconds)) {
            level = Math.max(level, pulseLevel);
        }
        if (DriverStation.isDisabled()) {
            level = 0.0;
        }
        heldLevel = 0.0;
        if (level != appliedLevel) {
            hid.setRumble(RumbleType.kBothRumble, level);
            appliedLevel = level;
        }
    }

    public double getLevel() {
        return Math.max(appliedLevel, 0.0);
    }
}
