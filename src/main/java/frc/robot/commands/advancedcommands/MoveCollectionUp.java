package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.subsystems.intake.CollectionMovement;

public class MoveCollectionUp extends SequentialCommandGroup {

    public MoveCollectionUp(CollectionMovement collectionMovement) {
        addCommands(
                new MoveCollection(collectionMovement, () -> 0.5)
        );
    }
}
