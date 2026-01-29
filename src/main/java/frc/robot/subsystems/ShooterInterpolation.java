package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

public class ShooterInterpolation {

    public static double getInterpolatedValue(double distanceMeters, double[] valueTable) {
        double[] distanceTable = ShooterConstants.DISTANCE_TABLE;

        // if it's too close
        if (distanceMeters <= distanceTable[0]) {
            return valueTable[0];
        }

        // if it's too far
        int maxIndex = distanceTable.length - 1;
        if (distanceMeters >= distanceTable[maxIndex]) {
            return valueTable[maxIndex];
        }

        int index = 0;
        for (int i = 0; i < maxIndex; i++) {
            if (distanceMeters >= distanceTable[i] && distanceMeters < distanceTable[i + 1]) {
                index = i;
                break;
            }
        }

        double x1 = distanceTable[index];
        double x2 = distanceTable[index + 1];
        double y1 = valueTable[index];
        double y2 = valueTable[index + 1];

        double percent = (distanceMeters - x1) / (x2 - x1);
        return y1 + (percent * (y2 - y1));
    }
}
