package frc.robot.commands.swerve;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.commands.RotateSwerveWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;

import java.util.function.Supplier;

public class RotateAccordingToGyro extends RotateSwerveWithPID {

    private static final RootNamespace namespace = new RootNamespace("rotate swerve");

    private static final PIDSettings rotatePIDSettings = namespace.addPIDNamespace("gyro",
            new PIDSettings(0.037, 0.0, 0.0001537, 0, 0.3, 0.5));

    private static final FeedForwardSettings rotateFeedForwardSettings = namespace.addFeedForwardNamespace(
            "gyro", new FeedForwardSettings(FeedForwardController.ControlMode.LINEAR_POSITION));

    public RotateAccordingToGyro(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> xSpeed,
                                 Supplier<Double> ySpeed, boolean useVelocityPID) {
        super(drivetrain, setpoint, () -> drivetrain.getAngle().getDegrees(), xSpeed, ySpeed, rotatePIDSettings,
                rotateFeedForwardSettings, useVelocityPID);
    }

    public RotateAccordingToGyro(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, boolean useVelocityPID){
        super(drivetrain, setpoint, () -> drivetrain.getAngle().getDegrees(), rotatePIDSettings,
                rotateFeedForwardSettings, useVelocityPID);
    }
}
