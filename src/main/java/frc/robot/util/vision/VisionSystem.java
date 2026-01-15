package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VisionSystem {

    private final List<AprilTagCamera> cameras;

    public VisionSystem(AprilTagCamera... cameras) {
        this.cameras = List.of(cameras);
    }

    public List<VisionMeasurement> getMeasurements(ChassisSpeeds currentSpeeds) {
        List<VisionMeasurement> validMeasurements = new ArrayList<>();

        for (AprilTagCamera camera : cameras) {
            if (!camera.isConnected()) continue;
            validMeasurements.addAll(camera.getMeasurements(currentSpeeds));
        }

        return validMeasurements;
    }
}
