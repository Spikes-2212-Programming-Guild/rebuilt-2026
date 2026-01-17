package frc.robot.util.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * formerly "VisionConstants".
 * Centralizes tuning constants and math helpers for the vision system.
 */
public class VisionConstants {

    public static final double MAX_DRIVE_SPEED_M_S = -1;
    public static final double MAX_TURN_SPEED_RAD_S = -1;

    public static final double MAX_TAG_DISTANCE_METERS = -1;

    // Increase these numbers to trust vision LESS. Decrease to trust MORE.
    // Drive 0.05 = ~5cm trust at 1 meter. Trust falls off with distance squared.
    public static final double DRIVE_TRUST_SCALAR = -1;
    public static final double ANGLE_TRUST_SCALAR = -1;

    /**
     * Calculates a standard deviation that scales with distance.
     *
     * @param scalingFactor the baseline trust scalar (e.g. DRIVE_TRUST_SCALAR)
     * @param distance      the distance to the target in meters
     * @return The standard deviation to pass to the pose estimator
     */
    public static double calculateStandardDeviation(double scalingFactor, double distance) {
        return scalingFactor * Math.pow(distance, 2);
    }

    /**
     * Checks if a measurement is reliable enough to be used.
     */
    public static boolean isReliable(double averageTagDistance, ChassisSpeeds speeds) {
        if (averageTagDistance >= MAX_TAG_DISTANCE_METERS) return false;

        double driveVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(speeds.omegaRadiansPerSecond);

        return driveVelocity <= MAX_DRIVE_SPEED_M_S && turnVelocity <= MAX_TURN_SPEED_RAD_S;
    }
}
