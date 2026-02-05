package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelQuadratic;

public class ShooterAlgo {

    public static FlywheelQuadratic getOptimalPose(double distanceMeters) {
        if (distanceMeters < ShooterConstants.STRATEGY.SWITCH_POINT_1_TO_2) {
            return FlywheelQuadratic.SHOOT_POSE1; // Close Range
        }
        else if (distanceMeters < ShooterConstants.STRATEGY.SWITCH_POINT_2_TO_3) {
            return FlywheelQuadratic.SHOOT_POSE2; // Mid Range
        }
        else {
            return FlywheelQuadratic.SHOOT_POSE3; // Long Range
        }
    }

    public static double calculateRPM(double distanceMeters, FlywheelQuadratic hoodPose) {
        return (hoodPose.flywheelQuadraticVector * Math.pow(distanceMeters, 2))
                + (hoodPose.flywheelLinearVector * distanceMeters)
                + hoodPose.flywheelInterceptVector;
    }
}
