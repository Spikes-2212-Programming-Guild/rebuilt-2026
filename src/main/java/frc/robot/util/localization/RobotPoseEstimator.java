package frc.robot.util.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.odometry.OdometryManager;
import frc.robot.util.odometry.OdometryMeasurement;
import frc.robot.util.odometry.PeriodicTaskScheduler;
import frc.robot.util.odometry.StandardDeviations;
import frc.robot.util.vision.VisionMeasurement;
import frc.robot.util.vision.VisionSystem;

import java.util.function.Supplier;

public class RobotPoseEstimator {

    private static final StandardDeviations ODOMETRY_STD_DEVS =
            new StandardDeviations(-1, -1);

    private final SwerveDrivePoseEstimator poseEstimator;
    private final OdometryManager odometryManager;
    private final VisionSystem visionSystem;

    public RobotPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
                              SwerveModulePosition[] modulePositions, Pose2d initPose,
                              Supplier<OdometryMeasurement> odometryMeasurementSupplier,
                              PeriodicTaskScheduler taskScheduler, VisionSystem visionSystem) {

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroAngle, modulePositions, initPose,
                ODOMETRY_STD_DEVS.toMatrix(), VecBuilder.fill(0, 0, 0)
        );

        this.odometryManager = new OdometryManager(
                this::addOdometryMeasurement, odometryMeasurementSupplier, taskScheduler
        );

        this.visionSystem = visionSystem;
    }

    public void periodic() {
        odometryManager.applyMeasurements();
        // TODO - update the vision part once it's complete
        for (VisionMeasurement measurement : visionSystem.getMeasurements(null)) {
            addVisionMeasurement(measurement);
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPoseByLatency(ChassisSpeeds relativeSpeeds, double latencySeconds) {
        double predictedX = relativeSpeeds.vxMetersPerSecond * latencySeconds;
        double predictedY = relativeSpeeds.vyMetersPerSecond * latencySeconds;
        Rotation2d predictedRotation = Rotation2d.fromRadians(relativeSpeeds.omegaRadiansPerSecond * latencySeconds);
        return getEstimatedPose().transformBy(new Transform2d(predictedX, predictedY, predictedRotation));
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPose(newPose);
    }

    public SwerveDrivePoseEstimator getEstimator() {
        return poseEstimator;
    }

    public void addOdometryMeasurement(OdometryMeasurement measurement) {
        if (measurement == null) return;
        poseEstimator.updateWithTime(
            measurement.timestampSeconds(),
            measurement.robotYaw(),
            measurement.wheelPositions()
        );
    }

    public void addVisionMeasurement(VisionMeasurement measurement) {
        if (measurement == null) return;
        poseEstimator.addVisionMeasurement(
                measurement.estimatedPose(),
                measurement.timestampSeconds(),
                measurement.standardDeviations()
        );
    }
}
