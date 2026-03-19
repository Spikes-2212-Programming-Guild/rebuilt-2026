package frc.robot.commands.swerve;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.commands.RotateSwerveWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class RotateAccordingAprilTags extends RotateSwerveWithPID {

    private static final RootNamespace namespace = new RootNamespace("rotate according april tag");

    private static final PIDSettings rotatePIDSettings = namespace.addPIDNamespace("april tag",
            new PIDSettings(0.025, 0.001, 0.0, 0, 1.5, 0.5));

    private static final FeedForwardSettings rotateFeedForwardSettings = namespace.addFeedForwardNamespace(
            "april tag", new FeedForwardSettings(FeedForwardController.ControlMode.LINEAR_POSITION));

    public RotateAccordingAprilTags(SwerveDrivetrain drivetrain, Supplier<Double> setpoint,
                                    VisionService visionService, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                                    Boolean useVelocityPID) {
        super(drivetrain, setpoint, () -> visionService.getX() * -1, xSpeed, ySpeed, rotatePIDSettings,
                rotateFeedForwardSettings, useVelocityPID);
    }

    public RotateAccordingAprilTags(SwerveDrivetrain drivetrain, Supplier<Double> setpoint,
                                    VisionService visionService, Boolean useVelocityPID) {
        super(drivetrain, setpoint, visionService::getX, rotatePIDSettings,
                rotateFeedForwardSettings, useVelocityPID);
    }
}
