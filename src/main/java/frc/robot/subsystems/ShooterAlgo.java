package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.HoodPose;

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
        return (hoodPose.flywheelQuadraticVector * Math.pow(distanceMeters, 2))
                + (hoodPose.flywheelLinearVector * distanceMeters)
                + hoodPose.flywheelInterceptVector;
    }
}
