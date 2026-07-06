package frc.robot.commands.shooter;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

public class ShootWithPID extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("shoot with pid cmd");

    private static final PIDSettings PID_SETTINGS =
            new PIDSettings(0.3, 0.1, 0, 0, 0.25, 0);

    private static final FeedForwardSettings FEED_FORWARD_SETTINGS =
            new FeedForwardSettings(0.077, 0.097, 0, FeedForwardController.ControlMode.LINEAR_VELOCITY);

    public ShootWithPID(Supplier<Double> speed, double waitTime) {
        super(Shooter.getInstance(), new PIDSettings(
                        PID_SETTINGS.getkP(), PID_SETTINGS.getkI(), PID_SETTINGS.getkD(),
                        PID_SETTINGS.getIZone(), PID_SETTINGS.getTolerance(), waitTime
                ), FEED_FORWARD_SETTINGS,
                UnifiedControlMode.VELOCITY, speed, true);
    }

    public ShootWithPID(double speed, double waitTime) {
        super(Shooter.getInstance(), new PIDSettings(
                        PID_SETTINGS.getkP(), PID_SETTINGS.getkI(), PID_SETTINGS.getkD(),
                        PID_SETTINGS.getIZone(), PID_SETTINGS.getTolerance(), waitTime
                ), FEED_FORWARD_SETTINGS,
                UnifiedControlMode.VELOCITY, () -> speed, false);
    }

    @Override
    public void end(boolean interrupted) {

    }

    public static void updateNamespace() {
        namespace.update();
    }
}
