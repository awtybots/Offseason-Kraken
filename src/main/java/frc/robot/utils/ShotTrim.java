package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShotTrim {

    private static final LoggedNetworkNumber hub =
            new LoggedNetworkNumber("/SmartDashboard/Shooting/Hub RPM Multiplier", 1.0);
    private static final LoggedNetworkNumber ferry =
            new LoggedNetworkNumber("/SmartDashboard/Shooting/Ferry RPM Multiplier", 1.0);
    private static final LoggedNetworkNumber hubOffset =
            new LoggedNetworkNumber("/SmartDashboard/Shooting/Hub RPM Offset", ShooterConstants.HUB_RPM_OFFSET);
    private static final LoggedNetworkNumber ferryOffset =
            new LoggedNetworkNumber("/SmartDashboard/Shooting/Ferry RPM Offset", ShooterConstants.FERRY_RPM_OFFSET);

    private ShotTrim() {}

    public static double hub() {
        return MathUtil.clamp(hub.get(), ShooterConstants.RPM_TRIM_MIN, ShooterConstants.RPM_TRIM_MAX);
    }

    public static double ferry() {
        return MathUtil.clamp(ferry.get(), ShooterConstants.RPM_TRIM_MIN, ShooterConstants.RPM_TRIM_MAX);
    }

    public static double hubOffset() {
        return MathUtil.clamp(hubOffset.get(),
                -ShooterConstants.RPM_OFFSET_TRIM_LIMIT, ShooterConstants.RPM_OFFSET_TRIM_LIMIT);
    }

    public static double ferryOffset() {
        return MathUtil.clamp(ferryOffset.get(),
                -ShooterConstants.RPM_OFFSET_TRIM_LIMIT, ShooterConstants.RPM_OFFSET_TRIM_LIMIT);
    }

    public static double hubRPM(double tableRPM) {
        return tableRPM * hub() + hubOffset();
    }

    public static double ferryRPM(double tableRPM) {
        return tableRPM * ferry() + ferryOffset();
    }
}
