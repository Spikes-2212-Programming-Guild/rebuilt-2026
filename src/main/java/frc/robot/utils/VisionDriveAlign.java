package frc.robot.utils;

import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.Supplier;

public record VisionDriveAlign(SwerveDrivetrain swerve, Supplier<Double> forwardVelocity,
                               Supplier<Double> strafeVelocity) {

    private static final RootNamespace namespace = new RootNamespace("vision drive align");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("pid settings");

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

        swerve.drive(forwardVelocity.get(), strafeVelocity.get(), rotation, isFieldRelative, useVelocityPID);
    }

    public PIDController getController() {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }
}
