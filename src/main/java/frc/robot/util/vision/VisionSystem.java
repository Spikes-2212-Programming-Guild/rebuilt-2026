package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.cameras.AprilTagCamera;

public class VisionSystem {

    private final List<AprilTagCamera> cameras;

    public VisionSystem(AprilTagCamera... cameras) {
        this.cameras = List.of(cameras);
    }

    /**
     * Polls all cameras for new data.
     * @param robotRelativeSpeeds Used to check if the robot is moving too fast for clear images.
     * @return A list of valid measurements from all cameras.
     */
    public List<VisionMeasurement> getMeasurements(ChassisSpeeds robotRelativeSpeeds) {
        List<VisionMeasurement> validMeasurements = new ArrayList<>();

        for (AprilTagCamera camera : cameras) {
            if (!camera.isConnected()) continue;
            validMeasurements.addAll(camera.getMeasurements(robotRelativeSpeeds));
        }

        return validMeasurements;
    }
}
