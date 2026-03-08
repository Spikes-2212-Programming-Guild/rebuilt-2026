package frc.robot.utils;

import frc.robot.Constants.ShooterConstants;

public class ShooterAlgorithm {

    public static double calculateRPM(double distanceMeters) {
        return (ShooterConstants.A * Math.pow(distanceMeters, 2))
                + (ShooterConstants.B * distanceMeters)
                + ShooterConstants.C;
    }
}
