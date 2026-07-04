package frc.robot.commands.shoot;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

public class ShootWithPID extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("shoot with pid command");

    private static final PIDSettings PID_SETTINGS = new PIDSettings(
            0.2, 0.001, 0.009, 0, 0.5, 0.1
    );
//    private static final PIDSettings PID_SETTINGS = namespace.
//            addPIDNamespace("shoot", new PIDSettings(0.2, 0.001, 0.009,
//                    0, 0.5, 0.1));

    private static final FeedForwardSettings FEED_FORWARD_SETTINGS = new FeedForwardSettings(
            0.0395, 0.115, 0, FeedForwardController.ControlMode.LINEAR_VELOCITY
    );
//    private static final FeedForwardSettings FEED_FORWARD_SETTINGS = namespace.
//            addFeedForwardNamespace("shoot", new FeedForwardSettings(0.0395, 0.115, 0,
//                    FeedForwardController.ControlMode.LINEAR_VELOCITY));

    public ShootWithPID(Supplier<Double> speed, double waitTime) {
        super(Shooter.getInstance(), new PIDSettings(PID_SETTINGS.getkP(), PID_SETTINGS.getkI(), PID_SETTINGS.getkD(),
                        PID_SETTINGS.getIZone(), PID_SETTINGS.getTolerance(), waitTime), FEED_FORWARD_SETTINGS,
                UnifiedControlMode.VELOCITY, speed, false);
        namespace.putNumber("setpoint", speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    public static void updateNamespace() {
        namespace.update();
    }
}
