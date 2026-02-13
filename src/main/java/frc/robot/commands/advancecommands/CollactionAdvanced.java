package frc.robot.commands.advancecommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

public class CollactionAdvanced extends SequentialCommandGroup {

    public CollactionAdvanced(Collection collection, CollectionMovement collectionMovement) {
        addCommands(new SequentialCommandGroup(
                new CollectionToPosition(collectionMovement)),
        new CollactionAdvanced(collection, collectionMovement));

    }
}
