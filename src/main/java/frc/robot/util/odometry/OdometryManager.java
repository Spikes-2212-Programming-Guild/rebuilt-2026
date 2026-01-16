package frc.robot.util.odometry;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class OdometryManager {

    public static final int UPDATE_FREQUENCY_HZ = 100;

    private static final int MEASUREMENT_OVERFLOW_LIMIT = -1;

    private final Queue<OdometryMeasurement> measurementsQueue = new ConcurrentLinkedQueue<>();
    private final Consumer<OdometryMeasurement> measurementConsumer;
    private final Supplier<OdometryMeasurement> measurementSupplier;

    public OdometryManager(Consumer<OdometryMeasurement> measurementConsumer,
                           Supplier<OdometryMeasurement> measurementSupplier,
                           PeriodicTaskScheduler taskScheduler) {

        this.measurementConsumer = measurementConsumer;
        this.measurementSupplier = measurementSupplier;
        taskScheduler.schedule(this::captureMeasurement, UPDATE_FREQUENCY_HZ, 0);
    }

    public void applyMeasurements() {
        while (!measurementsQueue.isEmpty()) {
            OdometryMeasurement measurement = measurementsQueue.poll();
            measurementConsumer.accept(measurement);
        }
    }

    private void captureMeasurement() {
        OdometryMeasurement measurement = measurementSupplier.get();
        if (measurement == null) return;

        if (measurementsQueue.size() >= MEASUREMENT_OVERFLOW_LIMIT)
            measurementsQueue.poll(); // drop the oldest if the queue is overflowed

        measurementsQueue.add(measurement);
    }
}
