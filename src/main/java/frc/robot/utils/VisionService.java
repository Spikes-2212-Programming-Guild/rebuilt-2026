package frc.robot.utils;

import com.spikes2212.util.Limelight;

public class VisionService {

    private static final String LIMELIGHT_NAME = "limelight";

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
        long id = limelight.getID();
//        boolean correctTag = (id == 26 || id == 24 || id == 27 || id == 9 || id == 10 || id == 8 || id == 11);
        if (hasTarget()) {
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
