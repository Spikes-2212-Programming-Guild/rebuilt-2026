package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class SimpleMoveCollectionToPosition extends MoveGenericSubsystem {

    private final CollectionMovement collection;

    public SimpleMoveCollectionToPosition(CollectionMovement collection, Supplier<Double> speed) {
        super(collection, speed);
        this.collection = collection;
    }

    @Override
    public void initialize() {
        collection.syncEncoder();
    }
}
