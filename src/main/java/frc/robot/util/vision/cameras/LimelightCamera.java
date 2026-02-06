package frc.robot.util.vision.cameras;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.vision.drivers.LimelightHelpers;
import frc.robot.util.vision.drivers.LimelightHelpers.PoseEstimate;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionConstants;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class LimelightCamera implements AprilTagCamera {

    private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;

    private final String limelightName;
    private final Supplier<Rotation2d> rotationSupplier;
    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = 0;

    public LimelightCamera(String name, Supplier<Rotation2d> rotationSupplier) {
        this.limelightName = name;
        this.rotationSupplier = rotationSupplier;
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
        updateRobotOrientation(robotSpeeds);
        var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (!isValid(pose, robotSpeeds)) {
            return Collections.emptyList();
        }

        return List.of(VisionConstants.createMeasurement(
                pose.pose,
                pose.timestampSeconds,
                pose.avgTagDist
        ));
    }

    private void updateRobotOrientation(ChassisSpeeds robotSpeeds) {
        double robotYawDegrees = rotationSupplier.get().getDegrees();
        double robotYawRateDegreesPerSec = Math.toDegrees(robotSpeeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                robotYawDegrees,
                robotYawRateDegreesPerSec,
                0, 0, 0, 0
        );
    }

    private boolean isValid(PoseEstimate pose, ChassisSpeeds robotSpeeds) {
        if (pose == null || pose.tagCount == 0) {
            return false;
        }
        return VisionConstants.isReliable(pose.avgTagDist, robotSpeeds);
    }

    @Override
    public String getName() {
        return limelightName;
    }
}
