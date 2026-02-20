package frc.robot.utils;

import frc.robot.utils.vision.RobotRelativeVisionSource;

public class RobotRelativeCollisionGuard {

    private static final double DANGER_AREA_THRESHOLD = -1.0;
    private final RobotRelativeVisionSource safetyCamera;

    public RobotRelativeCollisionGuard(RobotRelativeVisionSource safetyCamera) {
        this.safetyCamera = safetyCamera;
    }

    /**
     * Checks if the robot path is blocked by a large obstacle.
     */
    public boolean isPathBlocked() {
        if (safetyCamera.hasValidTarget()) {
            return safetyCamera.getTargetArea() > DANGER_AREA_THRESHOLD;
        }
        return false;
    }
}
