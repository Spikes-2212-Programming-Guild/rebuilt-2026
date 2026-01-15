package frc.robot.util.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

public interface AprilTagCamera {
    boolean isConnected();
    List<VisionMeasurement> getMeasurements(ChassisSpeeds robotSpeeds);
    String getName();
}
