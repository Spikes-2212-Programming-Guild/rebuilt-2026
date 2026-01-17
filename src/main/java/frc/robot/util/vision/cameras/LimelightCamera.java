package frc.robot.util.vision.cameras;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.vision.drivers.LimelightHelpers;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionConstants;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class LimelightCamera implements AprilTagCamera {

    private static final double CONNECTION_TIMEOUT_SECONDS = -1.0;

    private final String limelightName;
    private final Supplier<Rotation2d> rotationSupplier;
    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = 0;

    /**
     * @param name The name of the limelight (e.g. "limelight-front")
     * @param rotationSupplier A supplier that returns the robot's current Gyro rotation (e.g. driveSubsystem::getRotation2d)
     */
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

        double robotYawDegrees = rotationSupplier.get().getDegrees();
        double robotYawRateDegreesPerSec = Math.toDegrees(robotSpeeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                robotYawDegrees,
                robotYawRateDegreesPerSec,
                0, 0, 0, 0
        );

        var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (pose == null || pose.tagCount == 0 || !VisionConstants.isReliable(pose.avgTagDist, robotSpeeds)) {
            return Collections.emptyList();
        }

        return List.of(new VisionMeasurement(
                pose.pose,
                pose.timestampSeconds,
                VecBuilder.fill(
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
