package frc.robot.util.vision;

import java.util.Optional;

public interface AprilTagCamera {
    boolean isConnected();
    Optional<VisionMeasurement> getLatestMeasurement();
    String getName();
}
