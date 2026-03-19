package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.subsystems.forbar.CollectionMovement;

public class MoveCollectionUpSlowly extends SequentialCommandGroup {

    private static final double TIME_TO_MOVE_FAST = 0.25;
    private static final double TIME_TO_MOVE_SLOW = 1.5;
    private static final double MOVE_FAST_SPEED = 0.25;
    private static final double MOVE_SLOW_SPEED = 0.12;

    public MoveCollectionUpSlowly(CollectionMovement collectionMovement) {
        addCommands(
                new MoveCollection(collectionMovement, () -> MOVE_FAST_SPEED).withTimeout(TIME_TO_MOVE_FAST),
                new MoveCollection(collectionMovement, () -> MOVE_SLOW_SPEED).withTimeout(TIME_TO_MOVE_SLOW)
        );
    }
}
