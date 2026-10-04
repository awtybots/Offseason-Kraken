package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShotTrim {

    private static final LoggedNetworkNumber hub =
            new LoggedNetworkNumber("/SmartDashboard/Shooting/Hub RPM Multiplier", 1.0);
    private static final LoggedNetworkNumber ferry =
            new LoggedNetworkNumber("/SmartDashboard/Shooting/Ferry RPM Multiplier", 1.0);

    private ShotTrim() {}

    public static double hub() {
        return MathUtil.clamp(hub.get(), ShooterConstants.RPM_TRIM_MIN, ShooterConstants.RPM_TRIM_MAX);
    }

    public static double ferry() {
        return MathUtil.clamp(ferry.get(), ShooterConstants.RPM_TRIM_MIN, ShooterConstants.RPM_TRIM_MAX);
    }
}
