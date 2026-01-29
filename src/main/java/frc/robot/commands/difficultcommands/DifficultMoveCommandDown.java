package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CollectionMovement;

public class DifficultMoveCommandDown extends MoveSmartMotorControllerGenericSubsystem {

    static double SETPOINTDOWN = -1;

    private static final RootNamespace namespace = new RootNamespace("move collection to position");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("move collection to position");
    private static final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("move collection to position",
            FeedForwardController.ControlMode.LINEAR_POSITION);

    private final CollectionMovement collectionMovement;

    public DifficultMoveCommandDown(CollectionMovement collectionMovement){
        super(collectionMovement , pidSettings, feedForwardSettings,
                UnifiedControlMode.POSITION,() -> SETPOINTDOWN,true);
        this.collectionMovement = collectionMovement;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || !subsystem.canMove(subsystem.getSpeed());
    }
}