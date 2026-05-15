package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collection.MoveCollectionJoint;
import frc.robot.subsystems.collection.CollectionJoint;

public class MoveCollectionUpSlowly extends SequentialCommandGroup {

    private static final double TIME_TO_MOVE_FAST = 0.35;
    private static final double TIME_TO_MOVE_SLOW = 1.2;
    private static final double MOVE_FAST_SPEED = 0.25;
    private static final double MOVE_SLOW_SPEED = 0.12;

    public MoveCollectionUpSlowly(CollectionJoint collectionJoint) {
        addCommands(
                new MoveCollectionJoint(collectionJoint, () -> MOVE_FAST_SPEED).withTimeout(TIME_TO_MOVE_FAST),
                new MoveCollectionJoint(collectionJoint, () -> MOVE_SLOW_SPEED).withTimeout(TIME_TO_MOVE_SLOW)
        );
    }
}
