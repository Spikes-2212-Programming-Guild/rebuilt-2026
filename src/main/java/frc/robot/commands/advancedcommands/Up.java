package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

public class Up extends SequentialCommandGroup {

    public Up(Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new ParallelCommandGroup(
                        new UpSlowly(collectionMovement),
                        new MoveGenericSubsystem(collection, () -> 1.0).withTimeout(1)
                )
        );
    }
}
