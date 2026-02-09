package frc.robot.util.odometry;

import com.spikes2212.dashboard.RootNamespace;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class OdometryManager {

    private static final RootNamespace namespace = new RootNamespace("odometry manager");

    private static final Supplier<Integer> UPDATE_FREQUENCY_HZ =
            namespace.addConstantInt("update frequency hz", 50);

    private static final Supplier<Integer> MEASUREMENT_OVERFLOW_LIMIT =
            namespace.addConstantInt("measurement overflow limit", 5);

    private final Queue<OdometryMeasurement> measurementsQueue = new ConcurrentLinkedQueue<>();
    private final Consumer<OdometryMeasurement> measurementConsumer;
    private final Supplier<OdometryMeasurement> measurementSupplier;

    public OdometryManager(Consumer<OdometryMeasurement> measurementConsumer,
                           Supplier<OdometryMeasurement> measurementSupplier,
                           PeriodicTaskScheduler taskScheduler) {

        this.measurementConsumer = measurementConsumer;
        this.measurementSupplier = measurementSupplier;
        taskScheduler.schedule(this::captureMeasurement, UPDATE_FREQUENCY_HZ.get(), 0);
    }

    public void applyMeasurements() {
        namespace.update();
        while (!measurementsQueue.isEmpty()) {
            OdometryMeasurement measurement = measurementsQueue.poll();
            measurementConsumer.accept(measurement);
        }
    }

    private void captureMeasurement() {
        OdometryMeasurement measurement = measurementSupplier.get();
        if (measurement == null) return;

        if (measurementsQueue.size() >= MEASUREMENT_OVERFLOW_LIMIT.get())
            measurementsQueue.poll(); // drop the oldest if the queue is overflowed

        measurementsQueue.add(measurement);
    }
}