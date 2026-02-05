package frc.robot;

public final class Constants {

    public static final class ShooterConstants {

        public enum FlywheelQuadratic {

            SHOOT_POSE1(new double[3], -1.0, -1.0, -1.0),
            SHOOT_POSE2(new double[3], -1.0, -1.0, -1.0),
            SHOOT_POSE3(new double[3], -1.0, -1.0, -1.0);

            public final double[] neededValues;
            public final double flywheelQuadraticVector;
            public final double flywheelLinearVector;
            public final double flywheelInterceptVector;

            FlywheelQuadratic(double[] neededValues,
                              double flywheelQuadraticVector,
                              double flywheelLinearVector,
                              double flywheelInterceptVector) {

                this.neededValues = neededValues;
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
