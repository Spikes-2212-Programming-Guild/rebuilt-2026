package frc.robot.util.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VisionService {

    private static final double MAX_DRIVE_SPEED = -1; // (m / s)
    private static final double MAX_TURN_SPEED = -1; // (rad / s)
    private static final double MAX_MEASUREMENT_DISTANCE = -1; // (m)

    public static double calculateStandardDeviation(double scalingFactor, double distance) {
        return scalingFactor * Math.pow(distance, 2);
    }

    public static boolean isReliable(double averageTagDistance, ChassisSpeeds speeds) {
        if (averageTagDistance >= MAX_MEASUREMENT_DISTANCE) return false;
        double driveVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(speeds.omegaRadiansPerSecond);
        return driveVelocity <= MAX_DRIVE_SPEED && turnVelocity <= MAX_TURN_SPEED;
    }
}
