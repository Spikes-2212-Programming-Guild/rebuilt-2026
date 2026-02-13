package frc.robot.utils;

import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveDrivetrain;
import java.util.function.Supplier;

public record VisionDriveAlign(SwerveDrivetrain swerve, Supplier<Double> vX, Supplier<Double> vY) {

    private static final RootNamespace namespace = new RootNamespace("vision drive align");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("pid settings");
    private static final double STICK_DEADBAND = -1.0;

    /**
     * Calculates the rotation speed based on Limelight data and executes the drive logic.
     *
     * @param controller the PIDController used for alignment
     * @param isFieldRelative whether the movement is field relative
     * @param useVelocityPID whether to use velocity PID for the drive motors
     */
    public void align(PIDController controller, boolean isFieldRelative, boolean useVelocityPID) {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        double rotation = 0;
        if (tv == 1.0) {
            rotation = controller.calculate(tx, 0);
        }

        double pureX = MathUtil.applyDeadband(vX.get(), STICK_DEADBAND);
        double pureY = MathUtil.applyDeadband(vY.get(), STICK_DEADBAND);

        swerve.drive(pureX, pureY, rotation, isFieldRelative, useVelocityPID);
    }

    public PIDController getController() {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }
}
