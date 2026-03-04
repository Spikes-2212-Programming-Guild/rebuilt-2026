package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class MoveCollectionWithPID extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("move collection with pid");

    private static final PIDSettings PID_SETTINGS = namespace.
            addPIDNamespace("difficult move collection", PIDSettings.EMPTY_PID_SETTINGS);

    private static final FeedForwardSettings FEED_FORWARD_SETTINGS = namespace.
            addFeedForwardNamespace("difficult move collection", FeedForwardSettings.EMPTY_FF_SETTINGS);

    public MoveCollectionWithPID(CollectionMovement collectionMovement,
                                 CollectionMovement.CollectionMovementPose collectionMovementPose) {
        super(collectionMovement, PID_SETTINGS, FEED_FORWARD_SETTINGS, UnifiedControlMode.POSITION,
                ()-> collectionMovementPose.neededPose, false);
    }

    public MoveCollectionWithPID(CollectionMovement collectionMovement, Supplier<Double> setpoint) {
        super(collectionMovement, PID_SETTINGS, FEED_FORWARD_SETTINGS, UnifiedControlMode.POSITION,
                setpoint, false);
    }
}
