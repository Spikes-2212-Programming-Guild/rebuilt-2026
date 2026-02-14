package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A placeholder for the robot's swerve drivetrain.
 */
public class SwerveDrivetrain extends SubsystemBase {

    private SwerveDrivetrain() {
        configureDashboard();
    }

    private static SwerveDrivetrain instance;

    public static SwerveDrivetrain getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrain();
        }
        return instance;
    }

    /**
     * Drives the robot using translation and rotation speeds.
     *
     * @param xSpeed          the speed in the x direction
     * @param ySpeed          the speed in the y direction
     * @param rotation        the rotation speed
     * @param isFieldRelative whether the movement is relative to the field
     * @param useVelocityPID  whether to use velocity PID for the motors
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean isFieldRelative, boolean useVelocityPID) {
    }

    public void configureDashboard() {
    }
}
