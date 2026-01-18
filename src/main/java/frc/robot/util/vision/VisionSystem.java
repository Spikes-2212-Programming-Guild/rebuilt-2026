package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.cameras.AprilTagCamera;

public class VisionSystem {

    private final List<AprilTagCamera> cameras;

    public VisionSystem(AprilTagCamera... cameras) {
        this.cameras = List.of(cameras);
    }

    /**
     * Polls all cameras for new data.
     *
     * @param robotRelativeSpeedSupplier Used to check if the robot is moving too fast for clear images.
     * @return A list of valid measurements from all cameras.
     */
    public List<VisionMeasurement> getMeasurements(Supplier<ChassisSpeeds> robotRelativeSpeedSupplier) {
        List<VisionMeasurement> validMeasurements = new ArrayList<>();

        ChassisSpeeds currentSpeeds = robotRelativeSpeedSupplier.get();

        for (AprilTagCamera camera : cameras) {
            if (!camera.isConnected()) continue;
            validMeasurements.addAll(camera.getMeasurements(currentSpeeds));
        }

        return validMeasurements;
    }
}
