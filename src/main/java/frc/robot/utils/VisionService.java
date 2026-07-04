package frc.robot.utils;

import com.spikes2212.util.Limelight;

import java.util.List;

public class VisionService {

    private static final String LIMELIGHT_NAME = "limelight";

    private static final List<Long> allowedTags = List.of(10L, 9L);

    private final Limelight limelight;

    private static VisionService instance;

    public static VisionService getInstance() {
        if (instance == null) {
            instance = new VisionService(LIMELIGHT_NAME);
        }
        return instance;
    }

    private VisionService(String limelightName) {
        limelight = new Limelight(limelightName);
    }

    public double getX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getZ() {
        if (hasTarget()
//                && allowedTags.contains(getTagId())
        ) {
            double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
            if (pose.length > 2) {
                return pose[2];
            }
        }
        return 0.0; //@TODO find a better default speed
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }

    public long getTagID() {
        return limelight.getID();
    }
}
