package frc.robot.util.odometry;

import com.spikes2212.dashboard.RootNamespace;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

public class OdometryBuffer {

    private static final RootNamespace namespace = new RootNamespace("odometry buffer");

    private static final Supplier<Integer> OVERFLOW_LIMIT =
            namespace.addConstantInt("overflow limit", 5);

    private final Queue<OdometryMeasurement> buffer = new ConcurrentLinkedQueue<>();

    public List<OdometryMeasurement> getMeasurements() {
        namespace.update();

        List<OdometryMeasurement> measurements = new ArrayList<>();
        while (!buffer.isEmpty()) {
            measurements.add(buffer.poll());
        }

        return measurements;
    }

    public void addMeasurement(OdometryMeasurement measurement) {
        if (measurement == null) return;

        if (buffer.size() >= OVERFLOW_LIMIT.get())
            buffer.poll(); // drop the oldest if the queue is overflowed

        buffer.add(measurement);
    }
}
