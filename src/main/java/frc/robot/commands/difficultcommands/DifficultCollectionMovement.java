package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class DifficultCollectionMovement extends MoveGenericSubsystem {

    private final CollectionMovement collectionMovement;

    public DifficultCollectionMovement(CollectionMovement collectionMovement, Supplier<Double> speed) {
        super(collectionMovement, speed);
        this.collectionMovement = collectionMovement;
    }

    public DifficultCollectionMovement(CollectionMovement collectionMovement,
                                       CollectionMovement.CollectionMovementPose pose) {
        this(collectionMovement, pose::getNeededSpeed);
    }
}
