package frc.robot.utils;

import frc.robot.utils.vision.RobotRelativeVisionSource;

public class RobotAvoidance {

    private static final double AVOIDANCE_AREA_THRESHOLD = -1.0;

    private final RobotRelativeVisionSource visionSensor;

    public RobotAvoidance(RobotRelativeVisionSource visionSensor) {
        this.visionSensor = visionSensor;
    }

    /**
     * Checks if the path is blocked by a large obstacle.
     *
     * @return true if the target area exceeds the safe threshold
     */
    public boolean isBlocked() {
        // If we see a target, and it is huge, it is likely an obstacle or we are too close
        if (visionSensor.hasValidTarget()) {
            return visionSensor.getTargetArea() > AVOIDANCE_AREA_THRESHOLD;
        }
        return false;
    }
}
