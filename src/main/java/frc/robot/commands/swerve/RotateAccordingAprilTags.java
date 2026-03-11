package frc.robot.commands.swerve;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.commands.RotateSwerveWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class RotateAccordingAprilTags extends RotateSwerveWithPID {

    private static final RootNamespace namespace = new RootNamespace("rotate according april tag");

    private static final PIDSettings rotatePIDSettings = namespace.addPIDNamespace("april tag",
            new PIDSettings(0.05, 0, 0.0003, 0, 0.3, 1));

    private static final FeedForwardSettings rotateFeedForwardSettings = namespace.addFeedForwardNamespace(
            "april tag", new FeedForwardSettings(0.335, 0, 0,
                    FeedForwardController.ControlMode.LINEAR_POSITION));

    public RotateAccordingAprilTags(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, VisionService visionService,
                                    Supplier<Double> xSpeed, Supplier<Double> ySpeed, Boolean useVelocityPID) {
        super(drivetrain, setpoint, visionService::getX, xSpeed, ySpeed, rotatePIDSettings,
                rotateFeedForwardSettings, useVelocityPID);
    }

    public RotateAccordingAprilTags(SwerveDrivetrain drivetrain, Supplier<Double> setpoint,
                                    VisionService visionService, Boolean useVelocityPID) {
        super(drivetrain, setpoint, visionService::getX, rotatePIDSettings,
                rotateFeedForwardSettings, useVelocityPID);
    }
 }
