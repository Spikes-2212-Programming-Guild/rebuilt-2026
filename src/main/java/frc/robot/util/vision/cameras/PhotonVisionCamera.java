package frc.robot.util.vision.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionConstants; // Import the new class
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonVisionCamera implements AprilTagCamera {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionCamera(String cameraName, AprilTagFieldLayout fieldLayout, Transform3d robotToCamera) {
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
    public List<VisionMeasurement> getMeasurements(ChassisSpeeds robotSpeeds) {
        var allResults = camera.getAllUnreadResults();
        List<VisionMeasurement> measurements = new ArrayList<>();

        for (var result : allResults) {
            var update = poseEstimator.update(result);
            if (update.isEmpty()) continue;

            var estimate = update.get();

            double avgDist = 0.0;
            if (!estimate.targetsUsed.isEmpty()) {
                for (PhotonTrackedTarget target : estimate.targetsUsed) {
                    avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                avgDist /= estimate.targetsUsed.size();
            }

            if (!VisionConstants.isReliable(avgDist, robotSpeeds)) continue;

            measurements.add(new VisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds,
                    VecBuilder.fill(
                            VisionConstants.calculateStandardDeviation(VisionConstants.DRIVE_TRUST_SCALAR, avgDist),
                            VisionConstants.calculateStandardDeviation(VisionConstants.DRIVE_TRUST_SCALAR, avgDist),
                            VisionConstants.calculateStandardDeviation(VisionConstants.ANGLE_TRUST_SCALAR, avgDist)
                    )
            ));
        }

        return measurements;
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
