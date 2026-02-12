package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class ShootWithPID extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("shoot with pid");

    private static final PIDSettings PID_SETTINGS = namespace.
            addPIDNamespace("difficult shoot", PIDSettings.EMPTY_PID_SETTINGS);

    private static final FeedForwardSettings FEED_FORWARD_SETTINGS = namespace.
            addFeedForwardNamespace("difficult shoot", FeedForwardSettings.EMPTY_FF_SETTINGS);

    public ShootWithPID(Shooter shooter, Supplier<Double> speed) {
        super(shooter, PID_SETTINGS, FEED_FORWARD_SETTINGS, UnifiedControlMode.VELOCITY, speed, false);
    }
}
