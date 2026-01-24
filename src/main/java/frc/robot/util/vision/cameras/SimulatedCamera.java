package frc.robot.util.vision.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class SimulatedCamera implements AprilTagCamera {

    private static final int SIM_RES_WIDTH = -1;
    private static final int SIM_RES_HEIGHT = -1;
    private static final double SIM_DIAGONAL_FOV_DEGREES = -1.0;
    private static final double SIM_AVG_ERROR_PX = -1.0;
    private static final double SIM_ERROR_STD_DEV_PX = -1.0;
    private static final double SIM_FPS = -1.0;
    private static final double SIM_AVG_LATENCY_MS = -1.0;

    private final String name;
    private final PhotonPoseEstimator poseEstimator;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final Supplier<Pose2d> simulatedRobotPoseSupplier;

    public SimulatedCamera(String name,
                           AprilTagFieldLayout layout,
                           Transform3d robotToCamera,
                           Supplier<Pose2d> simulatedRobotPoseSupplier) {
        this.name = name;
        this.simulatedRobotPoseSupplier = simulatedRobotPoseSupplier;

        PhotonCamera camera = new PhotonCamera(name);

        poseEstimator = new PhotonPoseEstimator(layout, robotToCamera);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(SIM_RES_WIDTH, SIM_RES_HEIGHT, Rotation2d.fromDegrees(SIM_DIAGONAL_FOV_DEGREES));
        props.setCalibError(SIM_AVG_ERROR_PX, SIM_ERROR_STD_DEV_PX);
        props.setFPS(SIM_FPS);
        props.setAvgLatencyMs(SIM_AVG_LATENCY_MS);

        visionSim = new VisionSystemSim(name);
        visionSim.addAprilTags(layout);

        cameraSim = new PhotonCameraSim(camera, props);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public boolean isConnected() {
        return true; // Sim is always connected
    }

    @Override
    public List<VisionMeasurement> getMeasurements(ChassisSpeeds robotRelativeSpeeds) {
        visionSim.update(simulatedRobotPoseSupplier.get());

        var results = cameraSim.getCamera().getAllUnreadResults();

        if (results.isEmpty()) return Collections.emptyList();

        var latestResult = results.get(results.size() - 1);
        var update = poseEstimator.estimateCoprocMultiTagPose(latestResult);

        if (update.isEmpty()) return Collections.emptyList();

        var estimate = update.get();
        double avgDist = 0.0;
        var targets = estimate.targetsUsed;
        if (!targets.isEmpty()) {
            for (var target : targets) avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            avgDist /= targets.size();
        }

        if (avgDist >= VisionConstants.MAX_TAG_DISTANCE_METERS) return Collections.emptyList();

        double driveVelocity = Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond);

        if (driveVelocity > VisionConstants.MAX_DRIVE_SPEED_MPS ||
                turnVelocity > VisionConstants.MAX_TURN_SPEED_RADPS) return Collections.emptyList();

        return List.of(new VisionMeasurement(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds,
                VecBuilder.fill(
                        VisionConstants.calculateStandardDeviation(VisionConstants.DRIVE_TRUST_SCALAR, avgDist),
                        VisionConstants.calculateStandardDeviation(VisionConstants.DRIVE_TRUST_SCALAR, avgDist),
                        VisionConstants.calculateStandardDeviation(VisionConstants.ANGLE_TRUST_SCALAR, avgDist)
                )
        ));
    }

    @Override
    public String getName() {
        return name;
    }
}
