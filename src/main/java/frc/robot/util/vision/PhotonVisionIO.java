package frc.robot.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class PhotonVisionIO implements VisionIO {

    private static final String CAMERA_NAME = "photonvision";

    private static final double STD_DEV_DRIVE_SCALING_FACTOR = -1;
    private static final double STD_DEV_ROTATION_SCALING_FACTOR = -1;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionIO(String cameraName, AprilTagFieldLayout fieldLayout, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera
        );
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public Optional<VisionMeasurement> getLatestMeasurement() {
        var results = camera.getAllUnreadResults();
        var result = poseEstimator.update(results.get(results.size() - 1));

        if (result.isEmpty()) return Optional.empty();

        var est = result.get();
        double estDistance = Math.hypot(est.estimatedPose.getX(), est.estimatedPose.getY());

        ChassisSpeeds speeds = null; //@TODO Retrieve real speeds from DriveSubsystem

        if (VisionService.isReliable(estDistance, speeds)) return Optional.empty();

        return Optional.of(new VisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                VecBuilder.fill(
                        VisionService.calculateStandardDeviation(STD_DEV_DRIVE_SCALING_FACTOR, estDistance),
                        VisionService.calculateStandardDeviation(STD_DEV_DRIVE_SCALING_FACTOR, estDistance),
                        VisionService.calculateStandardDeviation(STD_DEV_ROTATION_SCALING_FACTOR, estDistance)
                )
        ));
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
