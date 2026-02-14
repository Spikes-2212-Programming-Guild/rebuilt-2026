package frc.robot.utils;

import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.pathplanner.AutonomousContainer;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.Supplier;

import static frc.robot.pathplanner.AutonomousContainer.ROTATIONAL_PID_CONTROLLER;

public record VisionDriveAlign(SwerveDrivetrain swerve, Supplier<Double> xSpeed,
                               Supplier<Double> ySpeed) {

    /**
     * Calculates the rotation speed based on Limelight data and executes the drive logic.
     *
     * @param isFieldRelative whether the movement is field relative
     * @param useVelocityPID  whether to use velocity PID for the drive motors
     * @link controller the PIDController used for alignment
     */
    public void align(PIDController controller, boolean isFieldRelative, boolean useVelocityPID) {
        double tv = NetworkTableInstance.getDefault()
                .getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        double rotation = 0;
        if (tv == 1.0) {
            rotation = controller.calculate(tx, 0);
        }

        swerve.drive(xSpeed.get(), ySpeed.get(), rotation, isFieldRelative, useVelocityPID);
    }

    public PIDController getController() {
        return new PIDController(
                AutonomousContainer.ROTATIONAL_PID_CONTROLLER.getP(),
                AutonomousContainer.ROTATIONAL_PID_CONTROLLER.getI(),
                AutonomousContainer.ROTATIONAL_PID_CONTROLLER.getD()
        );
    }
}
