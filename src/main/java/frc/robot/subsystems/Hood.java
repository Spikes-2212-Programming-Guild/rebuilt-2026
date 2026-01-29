package frc.robot.subsystems;

public class Hood {

    public enum HoodPose {
        MAX_ANGLE(-1), MIN_ANGLE(0),
        SHOOT_POSE1(-1), SHOOT_POSE2(-1), SHOOT_POSE3(-1);

        public final double neededAngle;

        HoodPose(double neededPitch) {
            this.neededAngle = neededPitch;
        }
    }
}
