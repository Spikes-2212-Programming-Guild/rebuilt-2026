package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.Intake;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;

public class MoveCollectionUp extends SequentialCommandGroup {

    private static final int TIME_TO_COLLECT_WHEN_UP = 1;

    public MoveCollectionUp(Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new ParallelCommandGroup(
                        new MoveCollectionUpSlowly(collectionMovement),
                        new Intake(collection).withTimeout(TIME_TO_COLLECT_WHEN_UP)
                )
        );
    }
}
