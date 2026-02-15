package frc.robot.util.vision.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionConstants;
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
        poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
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
            var update = poseEstimator.estimateCoprocMultiTagPose(result);
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

            measurements.add(VisionConstants.createMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds,
                    avgDist
            ));
        }

        return measurements;
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
