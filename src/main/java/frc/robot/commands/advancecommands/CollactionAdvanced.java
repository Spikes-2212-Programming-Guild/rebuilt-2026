package frc.robot.commands.advancecommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.subsystems.CollectionMovement;

public class CollactionAdvanced extends SequentialCommandGroup {

    public CollactionAdvanced(CollectionMovement collectionMovement, MoveCollection moveCollection) {

        addCommands(
                new CollectionToPosition(collectionMovement, () -> -1.0),
                moveCollection);
    }
}
