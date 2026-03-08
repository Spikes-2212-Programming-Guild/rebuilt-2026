package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.intake.Intake;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;

import java.util.function.Supplier;

public class Collect extends ParallelCommandGroup {

    private static final Supplier<Double> DOWN_SPEED = ()-> -0.07;
    private static final int TIME_TO_MOVE_COLLECTION = 1;

    public Collect(Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new MoveCollection(collectionMovement, DOWN_SPEED).withTimeout(TIME_TO_MOVE_COLLECTION),
                new Intake(collection)
        );
    }
}
