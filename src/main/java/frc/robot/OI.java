package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.advancedcommands.MoveCollectionUp;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

public class OI {

    private final PlaystationControllerWrapper playstationControllerWrapper = new PlaystationControllerWrapper(0);

    private final Collection collection = Collection.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();

    public OI() {
        playstationControllerWrapper.getL2Button().whileTrue(new Collect(collection, collectionMovement))
                .onFalse(new MoveCollectionUp(collection, collectionMovement));
        }
}
