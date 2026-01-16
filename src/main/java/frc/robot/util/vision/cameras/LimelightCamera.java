package frc.robot.util.vision.cameras;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.vision.drivers.LimelightHelpers;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionConstants; // Import the new class

import java.util.Collections;
import java.util.List;

public class LimelightCamera implements AprilTagCamera {

    private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;
    private final String limelightName;
    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = 0;

    public LimelightCamera(String name, double yaw) {
        limelightName = name;
        // Initial orientation set.
        // NOTE: DriveSubsystem must call SetRobotOrientation periodically for MegaTag2 to work!
        LimelightHelpers.SetRobotOrientation(limelightName, yaw, 0, 0, 0, 0, 0);
    }

    @Override
    public boolean isConnected() {
        double currentHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (currentHeartbeat != lastHeartbeatValue) {
            lastHeartbeatValue = currentHeartbeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        }
        return (Timer.getFPGATimestamp() - lastHeartbeatTime) < CONNECTION_TIMEOUT_SECONDS;
    }

    @Override
    public List<VisionMeasurement> getMeasurements(ChassisSpeeds robotSpeeds) {
        // MegaTag2 Retrieval
        var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Validation using VisionConstants
        if (pose == null || pose.tagCount == 0 || !VisionConstants.isReliable(pose.avgTagDist, robotSpeeds)) {
            return Collections.emptyList();
        }

        return List.of(new VisionMeasurement(
                pose.pose,
                pose.timestampSeconds,
                VecBuilder.fill(
                        // Use centralized constants for trust
                        VisionConstants.calculateStandardDeviation(VisionConstants.DRIVE_TRUST_SCALAR, pose.avgTagDist),
                        VisionConstants.calculateStandardDeviation(VisionConstants.DRIVE_TRUST_SCALAR, pose.avgTagDist),
                        VisionConstants.calculateStandardDeviation(VisionConstants.ANGLE_TRUST_SCALAR, pose.avgTagDist)
                )
        ));
    }

    @Override
    public String getName() {
        return limelightName;
    }
}
