package frc.robot.util.localization;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.odometry.OdometryBuffer;
import frc.robot.util.odometry.OdometryMeasurement;
import frc.robot.util.odometry.StandardDeviations;
import frc.robot.util.odometry.TaskScheduler;

import java.util.function.Supplier;

public class RobotPoseEstimator {

    private static final RootNamespace namespace = new RootNamespace("robot pose estimator");

    private static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS =
            new StandardDeviations(
                namespace.addConstantInt("translation x", 0).get(),
                namespace.addConstantInt("translation y", 0).get(),
                namespace.addConstantInt("rotation", 0).get()
            );

    private static final Supplier<Integer> ODOMETRY_UPDATE_RATE =
            namespace.addConstantInt("odometry update rate", 50);

    private final SwerveDrivePoseEstimator poseEstimator;
    private final OdometryBuffer odometryBuffer;
    private final Supplier<OdometryMeasurement> odometryMeasurement;

    public RobotPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroYaw,
                              SwerveModulePosition[] modulePositions, Pose2d initRobotPose,
                              Supplier<OdometryMeasurement> odometryMeasurement) {

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroYaw, modulePositions, initRobotPose,
                ODOMETRY_STANDARD_DEVIATIONS.toMatrix(),
                StandardDeviations.EMPTY_STANDARD_DEVIATIONS.toMatrix()
        );

        this.odometryBuffer = new OdometryBuffer();
        this.odometryMeasurement = odometryMeasurement;
    }

    public void setupOdometryUpdateLoop(TaskScheduler scheduler) {
        scheduler.schedule(
                () -> this.odometryBuffer.addMeasurement(odometryMeasurement.get()),
                ODOMETRY_UPDATE_RATE.get(), 0
        );
    }

    public void periodic() {
        namespace.update();
        for (OdometryMeasurement measurement : odometryBuffer.getMeasurements()) {
            addOdometryMeasurement(measurement);
        }
        // @TODO add the vision part once it's complete
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatePoseWithLatency(ChassisSpeeds relativeSpeeds, double latencySeconds) {
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
