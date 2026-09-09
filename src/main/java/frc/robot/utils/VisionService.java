package frc.robot.utils;

import com.spikes2212.util.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionService {

    private static final String LIMELIGHT_NAME = "limelight";

    private final Limelight limelight;

    private static VisionService instance;

    private double lastZ = 0;

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
        if (hasTarget()) {
            double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
            if (pose.length > 2) {
                lastZ = pose[2];
            }
        }
        return lastZ;
    }

    public Pose2d getRobotPose() {
        double[] pose = limelight.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);

        if (pose.length < 6) {
            return null;
        }

        double x = pose[0];
        double y = pose[1];
        double yaw = pose[5];

        return new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
    }

    public double getLatencyMs() {
        double[] pose = limelight
                .getEntry("botpose_orb_wpiblue")
                .getDoubleArray(new double[0]);

        return pose.length > 6 ? pose[6] : 0.0;
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }

    public long getTagID() {
        return limelight.getID();
    }
}
