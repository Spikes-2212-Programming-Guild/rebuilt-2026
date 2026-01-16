package frc.robot.util.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryMeasurement(
        double timestamp,
        Rotation2d heading,
        SwerveModulePosition[] wheelPositions
) {}
