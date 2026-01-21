package frc.robot.util.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryMeasurement(
        double timestampSeconds,
        Rotation2d robotYaw,
        SwerveModulePosition[] wheelPositions
) {
}
