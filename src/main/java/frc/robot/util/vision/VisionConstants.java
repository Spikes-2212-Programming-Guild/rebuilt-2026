package frc.robot.util.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VisionConstants {

    public static final double MAX_DRIVE_SPEED_MPS = -1;
    public static final double MAX_TURN_SPEED_RADPS = -1; //@TODO chane it to Drivetrain when merging the codes
    public static final double MAX_TAG_DISTANCE_METERS = -1;

    public static final double DRIVE_TRUST_SCALAR = -1;
    public static final double ANGLE_TRUST_SCALAR = -1;

    public static double calculateStandardDeviation(double scalingFactor, double distance) {
        return scalingFactor * Math.pow(distance, 2);
    }

    public static boolean isReliable(double averageTagDistance, ChassisSpeeds speeds) {
        if (averageTagDistance >= MAX_TAG_DISTANCE_METERS) return false;

        double driveVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(speeds.omegaRadiansPerSecond);

        return driveVelocity <= MAX_DRIVE_SPEED_MPS && turnVelocity <= MAX_TURN_SPEED_RADPS;
    }

    /**
     * SHARED FACTORY METHOD
     * Creates a standard VisionMeasurement with calculated standard deviations.
     */
    public static VisionMeasurement createMeasurement(Pose2d pose, double timestamp, double avgDist) {
        return new VisionMeasurement(
                pose,
                timestamp,
                VecBuilder.fill(
                        calculateStandardDeviation(DRIVE_TRUST_SCALAR, avgDist), // X Standard Deviation (Meters)
                        calculateStandardDeviation(DRIVE_TRUST_SCALAR, avgDist), // Y Standard Deviation (Meters)
                        calculateStandardDeviation(ANGLE_TRUST_SCALAR, avgDist)  // Theta Standard Deviation (Radians)
                )
        );
    }
}
