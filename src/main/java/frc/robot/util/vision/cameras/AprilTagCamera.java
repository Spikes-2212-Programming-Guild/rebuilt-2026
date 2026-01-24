package frc.robot.util.vision.cameras;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.vision.VisionMeasurement;

import java.util.List;

public interface AprilTagCamera {

    boolean isConnected();

    List<VisionMeasurement> getMeasurements(ChassisSpeeds robotRelativeSpeeds);

    String getName();

}
