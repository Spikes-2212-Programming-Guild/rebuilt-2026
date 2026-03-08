package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.subsystems.CollectionMovement;

public class UpSlowly extends SequentialCommandGroup {

    public UpSlowly(CollectionMovement collectionMovement) {
        addCommands(
                new MoveCollection(collectionMovement, ()-> 0.18).withTimeout(0.25),
                new MoveCollection(collectionMovement, ()-> 0.12).withTimeout(1)
        );
    }
}
