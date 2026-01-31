package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.CollectionMovement;

public class DifficultCollectionMovement extends MoveGenericSubsystem {


    public DifficultCollectionMovement(CollectionMovement collectionMovement,
                                       CollectionMovement.CollectionMovementSpeed speed) {
        super(collectionMovement, speed::getNeededSpeed);
    }
}
