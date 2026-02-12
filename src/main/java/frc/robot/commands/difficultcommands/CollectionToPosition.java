package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class CollectionToPosition extends MoveSmartMotorControllerGenericSubsystem {


    private static final RootNamespace namespace = new RootNamespace("move to position");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("move to position");
    private static final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace
            ("move to position", FeedForwardController.ControlMode.LINEAR_POSITION);

    public CollectionToPosition(CollectionMovement collectionMovement, Supplier<Double> setpoint) {
        super(collectionMovement, pidSettings, feedForwardSettings,
                UnifiedControlMode.POSITION, setpoint, false);
    }
}
