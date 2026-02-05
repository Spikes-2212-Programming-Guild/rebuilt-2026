package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class DifficultShootWithPID extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("difficult shoot - pid");

    private static final PIDSettings pidSettings = namespace.
            addPIDNamespace("difficult shoot", PIDSettings.EMPTY_PID_SETTINGS);

    private static final FeedForwardSettings ffSettings = namespace.
            addFeedForwardNamespace("difficult shoot", FeedForwardSettings.EMPTY_FF_SETTINGS);

    public DifficultShootWithPID(Shooter shooter, Supplier<Double> speed) {
        super(shooter, pidSettings, ffSettings, UnifiedControlMode.VELOCITY, speed, true);
    }
}
