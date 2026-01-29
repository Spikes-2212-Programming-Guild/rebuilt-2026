package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;

public class MovingShotAlgo {

    public static ShotSolution calculateMovingShot(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

        Translation2d robotToHub = ShooterConstants.HUB_POSITION.minus(robotPose.getTranslation());
        Translation2d compensation = getTranslation2d(fieldRelativeSpeeds, robotToHub);
        Translation2d virtualHubPosition = ShooterConstants.HUB_POSITION.minus(compensation);

        Translation2d robotToVirtualHub = virtualHubPosition.minus(robotPose.getTranslation());
        double virtualDistance = robotToVirtualHub.getNorm();

        double targetRPM = ShooterInterpolation.getInterpolatedValue(
                virtualDistance,
                ShooterConstants.RPM_TABLE
        );

        double targetHoodAngle = ShooterInterpolation.getInterpolatedValue(
                virtualDistance,
                ShooterConstants.HOOD_ANGLE_TABLE
        );

        double targetHeading = Math.toDegrees(Math.atan2(robotToVirtualHub.getY(), robotToVirtualHub.getX()));

        return new ShotSolution(targetRPM, targetHoodAngle, targetHeading);
    }

    private static Translation2d getTranslation2d(ChassisSpeeds fieldRelativeSpeeds, Translation2d robotToHub) {
        double rawDistance = robotToHub.getNorm();

        double timeOfFlight = ShooterInterpolation.getInterpolatedValue(
                rawDistance,
                ShooterConstants.TIME_OF_FLIGHT_TABLE
        );

        Translation2d robotVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        return robotVelocity.times(timeOfFlight);
    }
}
