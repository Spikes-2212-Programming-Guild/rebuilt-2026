package frc.robot.util.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class LimelightVisionIO implements VisionIO {

    private static final double STD_DEV_DRIVE_SCALING_FACTOR = -1;
    private static final double STD_DEV_ROTATION_SCALING_FACTOR = -1;

    private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;

    private final String limelightName;

    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = 0;

    public LimelightVisionIO(String name) {
        limelightName = name;
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
    public Optional<VisionMeasurement> getLatestMeasurement() {
        var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        ChassisSpeeds speeds = null; //@TODO Retrieve real speeds from DriveSubsystem
        if (pose == null || pose.tagCount == 0 || VisionService.isReliable(pose.avgTagDist, speeds)) {
            return Optional.empty();
        }

        return Optional.of(new VisionMeasurement(
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
