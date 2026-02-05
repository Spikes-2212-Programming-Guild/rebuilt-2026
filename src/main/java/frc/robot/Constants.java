package frc.robot;

public final class Constants {

    public static final class ShooterConstants {

        public enum HoodPose {

            SHOOT_POSE1(new double[]{1, 2, 3}, -1.0, -1.0, -1.0),
            SHOOT_POSE2(new double[]{1, 2, 3}, -1.0, -1.0, -1.0),
            SHOOT_POSE3(new double[]{1, 2, 3}, -1.0, -1.0, -1.0);

            public final double[] neededAngle;
            public final double flywheelQuadraticVector;
            public final double flywheelLinearVector;
            public final double flywheelInterceptVector;

            HoodPose(double[] neededAngle,
                     double flywheelQuadraticVector,
                     double flywheelLinearVector,
                     double flywheelInterceptVector) {
                this.neededAngle = neededAngle;
                this.flywheelQuadraticVector = flywheelQuadraticVector;
                this.flywheelLinearVector = flywheelLinearVector;
                this.flywheelInterceptVector = flywheelInterceptVector;
            }
        }

        public static final class STRATEGY {

            public static final double SWITCH_POINT_1_TO_2 = -1.0;
            public static final double SWITCH_POINT_2_TO_3 = -1.0;
        }
    }
}
