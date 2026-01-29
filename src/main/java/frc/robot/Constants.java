package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public static final class ShooterConstants {

        public static final Translation2d HUB_POSITION = new Translation2d(-1.0, -1.0);

        // Key: Distance in Meters (Must be sorted Low -> High)
        public static final double[] DISTANCE_TABLE = {-1.0, -1.0, -1.0};

        // Value: Hood Angle in Degrees
        public static final double[] HOOD_ANGLE_TABLE = {-1.0, -1.0, -1.0};

        // Value: Flywheel RPM
        public static final double[] RPM_TABLE = {-1.0, -1.0, -1.0};

        // Value: Time of Flight (Seconds)
        public static final double[] TIME_OF_FLIGHT_TABLE = {-1.0, -1.0, -1.0};
    }
}
