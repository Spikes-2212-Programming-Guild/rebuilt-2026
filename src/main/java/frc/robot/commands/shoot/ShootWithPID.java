package frc.robot.commands.shoot;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.shoot.Shooter;

import java.util.function.Supplier;

public class ShootWithPID extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("shoot with pid command");

    private static final PIDSettings PID_SETTINGS = namespace.
            addPIDNamespace("shoot", new PIDSettings(0.16, 0.0005, 0.006,
                    0, 0.1, 0));

    private static final FeedForwardSettings FEED_FORWARD_SETTINGS = namespace.
            addFeedForwardNamespace("shoot", new FeedForwardSettings(0.0395, 0.1, 0,
                    FeedForwardController.ControlMode.LINEAR_VELOCITY));

    public ShootWithPID(Shooter shooter, Supplier<Double> speed, double waitTime) {
        super(shooter, new PIDSettings(PID_SETTINGS.getkP(), PID_SETTINGS.getkI(), PID_SETTINGS.getkD(),
                        PID_SETTINGS.getIZone(), PID_SETTINGS.getTolerance(), waitTime), FEED_FORWARD_SETTINGS,
                UnifiedControlMode.VELOCITY, speed, true);
        namespace.putNumber("setpoint", speed);
    }

    public static void updateNamespace() {
        namespace.update();
    }
}
