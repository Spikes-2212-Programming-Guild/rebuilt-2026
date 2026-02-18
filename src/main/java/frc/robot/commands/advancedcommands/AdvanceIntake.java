package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

public class AdvanceIntake extends SequentialCommandGroup {

    public AdvanceIntake(Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new CollectionToPosition(collectionMovement, () -> CollectionMovement.OPEN_POSE),
                new SimpleIntake(collection)
        );
    }
}
