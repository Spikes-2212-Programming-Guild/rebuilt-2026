package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood.HoodPose;

public class ShooterAlgo {

    public static HoodPose getOptimalPose(double distanceMeters) {
        if (distanceMeters < ShooterConstants.STRATEGY.SWITCH_POINT_1_TO_2) {
            return HoodPose.SHOOT_POSE1; // Close Range
        }
        else if (distanceMeters < ShooterConstants.STRATEGY.SWITCH_POINT_2_TO_3) {
            return HoodPose.SHOOT_POSE2; // Mid Range
        }
        else {
            return HoodPose.SHOOT_POSE3; // Long Range
        }
    }

    public static double calculateRPM(double distanceMeters, HoodPose hoodPose) {
        double curve = 0;
        double slope = 0;
        double intercept = 0;

        switch (hoodPose) {
            case SHOOT_POSE1:
                curve = ShooterConstants.POSE1.FLYWHEEL_QUADRATIC_VECTOR;
                slope = ShooterConstants.POSE1.FLYWHEEL_LINEAR_VECTOR;
                intercept = ShooterConstants.POSE1.FLYWHEEL_INTERCEPT_VECTOR;
                break;

            case SHOOT_POSE2:
                curve = ShooterConstants.POSE2.FLYWHEEL_QUADRATIC_VECTOR;
                slope = ShooterConstants.POSE2.FLYWHEEL_LINEAR_VECTOR;
                intercept = ShooterConstants.POSE2.FLYWHEEL_INTERCEPT_VECTOR;
                break;

            case SHOOT_POSE3:
                curve = ShooterConstants.POSE3.FLYWHEEL_QUADRATIC_VECTOR;
                slope = ShooterConstants.POSE3.FLYWHEEL_LINEAR_VECTOR;
                intercept = ShooterConstants.POSE3.FLYWHEEL_INTERCEPT_VECTOR;
                break;

            default:
        }

        return (curve * Math.pow(distanceMeters, 2)) + (slope * distanceMeters) + intercept;
    }
}
