package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collection.SpinRoller;
import frc.robot.subsystems.collection.CollectionJoint;
import frc.robot.subsystems.collection.Roller;

public class MoveCollectionUp extends SequentialCommandGroup {

    private static final int TIME_TO_COLLECT_WHEN_UP = 1;

    public MoveCollectionUp(Roller roller, CollectionJoint collectionJoint) {
        addCommands(
                new ParallelCommandGroup(
                        new MoveCollectionUpSlowly(collectionJoint),
                        new SpinRoller(roller).withTimeout(TIME_TO_COLLECT_WHEN_UP)
                )
        );
    }
}
