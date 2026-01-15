package frc.robot.util.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class LimelightCamera implements AprilTagCamera {

    private static final double STD_DEV_DRIVE_SCALING_FACTOR = -1;
    private static final double STD_DEV_ROTATION_SCALING_FACTOR = -1;

    private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;

    private final String limelightName;

    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = 0;

    public LimelightCamera(String name, double yaw) {
        limelightName = name;
        LimelightHelpers.SetRobotOrientation(
                limelightName, yaw, 0, 0, 0, 0, 0
        );
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
        var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (pose == null || pose.tagCount == 0 || !VisionService.isReliable(pose.avgTagDist, robotSpeeds)) {
            return Collections.emptyList();
        }

        return List.of(new VisionMeasurement(
                pose.pose,
                pose.timestampSeconds,
                VecBuilder.fill(
                        VisionService.calculateStandardDeviation(STD_DEV_DRIVE_SCALING_FACTOR, pose.avgTagDist),
                        VisionService.calculateStandardDeviation(STD_DEV_DRIVE_SCALING_FACTOR, pose.avgTagDist),
                        VisionService.calculateStandardDeviation(STD_DEV_ROTATION_SCALING_FACTOR, pose.avgTagDist)
                )
        ));
    }

    @Override
    public String getName() {
        return limelightName;
    }
}
