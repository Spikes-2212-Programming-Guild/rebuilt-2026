package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.cameras.AprilTagCamera;

public class VisionSystem {

    private final List<AprilTagCamera> cameras;
    private final Supplier<ChassisSpeeds> speedSupplier;

    public VisionSystem(Supplier<ChassisSpeeds> speedSupplier, AprilTagCamera... cameras) {
        this.speedSupplier = speedSupplier;
        this.cameras = List.of(cameras);
    }

    public List<VisionMeasurement> getMeasurements() {
        List<VisionMeasurement> validMeasurements = new ArrayList<>();

        ChassisSpeeds currentSpeeds = speedSupplier.get();

        for (AprilTagCamera camera : cameras) {
            if (!camera.isConnected()) continue;
            validMeasurements.addAll(camera.getMeasurements(currentSpeeds));
        }

        return validMeasurements;
    }
}
