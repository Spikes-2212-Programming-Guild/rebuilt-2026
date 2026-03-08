package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

public class Collect extends SequentialCommandGroup {

    public Collect(Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new ParallelCommandGroup(
                    new MoveCollection(collectionMovement, () -> -0.05).withTimeout(1),
                    new SimpleIntake(collection)
                )
        );
    }
}
