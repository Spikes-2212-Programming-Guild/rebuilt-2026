package frc.robot.util.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

public class OdometryManager {

    public static final int UPDATE_FREQUENCY_HZ = 100;

    private static final int MEASUREMENT_OVERFLOW_LIMIT = -1;

    private final Queue<OdometryMeasurement> measurementsQueue = new ConcurrentLinkedQueue<>();
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<OdometryMeasurement> measurementSupplier;

    public OdometryManager(SwerveDrivePoseEstimator poseEstimator,
                           Supplier<OdometryMeasurement> measurementSupplier,
                           PeriodicTaskScheduler taskScheduler) {
        this.poseEstimator = poseEstimator;
        this.measurementSupplier = measurementSupplier;
        taskScheduler.schedule(this::recordMeasurement, UPDATE_FREQUENCY_HZ, 0);
    }

    public void update() {
        while (!measurementsQueue.isEmpty()) {
            OdometryMeasurement m = measurementsQueue.poll();
            poseEstimator.updateWithTime(m.timestamp(), m.heading(), m.wheelPositions());
        }
    }

    private void recordMeasurement() {
        OdometryMeasurement measurement = measurementSupplier.get();
        if (measurement == null) return;

        if (measurementsQueue.size() >= MEASUREMENT_OVERFLOW_LIMIT)
            measurementsQueue.poll(); // drop the oldest if the queue is overflowed

        measurementsQueue.add(measurement);
    }

    public void resetPose(Pose2d newPose) {
        OdometryMeasurement measurement = measurementSupplier.get();
        if (measurement == null) return;
        poseEstimator.resetPosition(
                measurement.heading(),
                measurement.wheelPositions(),
                newPose
        );
        measurementsQueue.clear();
    }
}
