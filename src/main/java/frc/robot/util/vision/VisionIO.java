package frc.robot.util.vision;

import java.util.Optional;

public interface VisionIO {
    boolean isConnected();
    Optional<VisionMeasurement> getLatestMeasurement();
    String getName();
}