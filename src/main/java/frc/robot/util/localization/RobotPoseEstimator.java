package frc.robot.util.localization;

import com.spikes2212.dashboard.RootNamespace;
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

import java.util.function.Supplier;

public class RobotPoseEstimator {

    private static final RootNamespace namespace = new RootNamespace("robot pose estimator");

    private static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS =
            new StandardDeviations(
                namespace.addConstantInt("translation x", 0).get(),
                namespace.addConstantInt("translation y", 0).get(),
                namespace.addConstantInt("rotation", 0).get()
            );

    private final SwerveDrivePoseEstimator poseEstimator;
    private final OdometryManager odometryManager;

    public RobotPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroYaw,
                              SwerveModulePosition[] modulePositions, Pose2d initRobotPose,
                              Supplier<OdometryMeasurement> odometryMeasurement,
                              PeriodicTaskScheduler taskScheduler) {

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroYaw, modulePositions, initRobotPose,
                ODOMETRY_STANDARD_DEVIATIONS.toMatrix(),
                StandardDeviations.EMPTY_STANDARD_DEVIATIONS.toMatrix()
        );

        this.odometryManager = new OdometryManager(
                this::addOdometryMeasurement, odometryMeasurement, taskScheduler
        );
    }

    public void periodic() {
        namespace.update();
        odometryManager.applyMeasurements();
        // @TODO add the vision part once it's complete
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPoseByLatency(ChassisSpeeds relativeSpeeds, double latencySeconds) {
        double predictedX = relativeSpeeds.vxMetersPerSecond * latencySeconds;
        double predictedY = relativeSpeeds.vyMetersPerSecond * latencySeconds;
        Rotation2d predictedRotation =
                Rotation2d.fromRadians(relativeSpeeds.omegaRadiansPerSecond * latencySeconds);
        return getEstimatedPose().transformBy(new Transform2d(predictedX, predictedY, predictedRotation));
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
}
