package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collection.MoveCollectionJoint;
import frc.robot.subsystems.collection.CollectionJoint;

public class Jumpies extends SequentialCommandGroup {

    private static final double UP_SPEED = 0.25;
    private static final double DOWN_SPEED = -0.1;

    public Jumpies(CollectionJoint collectionJoint) {
        addCommands(
                new MoveCollectionJoint(collectionJoint, () -> UP_SPEED).withTimeout(0.2),
                new MoveCollectionJoint(collectionJoint, () -> DOWN_SPEED).withTimeout(0.3)
        );
    }
}
