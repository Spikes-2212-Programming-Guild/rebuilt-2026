package frc.robot.commands.advancecommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.CollectionMovement;

public class Collection extends SequentialCommandGroup {

    public Collection(frc.robot.subsystems.Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new CollectionToPosition(collectionMovement, () -> CollectionMovement.OPEN_POSE),
                new SimpleIntake(collection)
        );
    }
}
