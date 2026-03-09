package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class Collect extends ParallelCommandGroup {

    private static final Supplier<Double> DOWN_SPEED = ()-> -0.07;
    private static final int TIME_TO_MOVE_COLLECTION = 1;

    public Collect(Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new MoveCollection(collectionMovement, DOWN_SPEED).withTimeout(TIME_TO_MOVE_COLLECTION),
                new SimpleIntake(collection)
        );
    }
}
