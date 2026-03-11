package frc.robot.commands.swerve;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class RotateAccordingAprilTags extends Command {

    private static final RootNamespace namespace = new RootNamespace("rotate according april tag");

    private static final PIDSettings rotatePIDSettings = namespace.addPIDNamespace("rotate",
            PIDSettings.EMPTY_PID_SETTINGS);
    private static final FeedForwardSettings rotateFeedForwardSettings = namespace.addFeedForwardNamespace(
            "rotate", new FeedForwardSettings(FeedForwardController.ControlMode.LINEAR_POSITION));

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> setpoint;
    private final VisionService visionService;

    protected final PIDController pidController;
    protected final FeedForwardController feedForwardController;

    protected final Supplier<Double> xSpeed;
    protected final Supplier<Double> ySpeed;

    protected double lastGivenTime;
    protected double now;
    protected double lastTimeNotOnTarget;

    public RotateAccordingAprilTags(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, VisionService visionService,
                                    Supplier<Double> xSpeed, Supplier<Double> ySpeed) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.visionService = visionService;
        this.setpoint = setpoint;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;

        pidController = new PIDController(0.05, 0, 0.0003);
        pidController.setIZone(0);
        pidController.setTolerance(0.3);

        feedForwardController = new FeedForwardController(new FeedForwardSettings(0.335, 0, 0 , FeedForwardController.ControlMode.LINEAR_POSITION));
        lastTimeNotOnTarget = 0;
    }

    public RotateAccordingAprilTags(SwerveDrivetrain drivetrain, Supplier<Double> setpoint,
                                    VisionService visionService) {
        this(drivetrain, setpoint, visionService, () -> 0.0, () -> 0.0);
    }

    @Override
    public void initialize() {
        lastGivenTime = Timer.getFPGATimestamp();
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        now = Timer.getFPGATimestamp();
        feedForwardController.setGains(rotateFeedForwardSettings);
        pidController.setPID(rotatePIDSettings.getkP(), rotatePIDSettings.getkI(), rotatePIDSettings.getkD());
        pidController.setIZone(rotatePIDSettings.getIZone());
        pidController.setTolerance(rotatePIDSettings.getTolerance());
        drivetrain.drive(xSpeed.get(), ySpeed.get(),pidController.calculate(
                -visionService.getX(), setpoint.get()) + feedForwardController.calculate(-visionService.getX(),
                setpoint.get()), false, now - lastGivenTime, true);
        lastGivenTime = now;
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return rotatePIDSettings.getWaitTime() <= now - lastTimeNotOnTarget;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
