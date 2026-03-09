package frc.robot.commands.swerve;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.commands.RotateSwerveWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;

import java.util.function.Supplier;

public class SwerveRotateWithPID extends RotateSwerveWithPID {

    private static final RootNamespace namespace = new RootNamespace("rotate swerve");

    private static final PIDSettings rotatePIDSettings = namespace.addPIDNamespace("rotate",
            PIDSettings.EMPTY_PID_SETTINGS);
    private static final FeedForwardSettings rotateFeedForwardSettings = namespace.addFeedForwardNamespace(
            "rotate", new FeedForwardSettings(FeedForwardController.ControlMode.LINEAR_VELOCITY));

    public SwerveRotateWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> xSpeed,
                               Supplier<Double> ySpeed) {
        super(drivetrain, setpoint, xSpeed, ySpeed, rotatePIDSettings, rotateFeedForwardSettings);
    }

    public SwerveRotateWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint){
        super(drivetrain, setpoint, rotatePIDSettings, rotateFeedForwardSettings);
    }
}
