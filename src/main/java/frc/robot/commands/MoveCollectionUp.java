package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class MoveCollectionUp extends MoveGenericSubsystem {

    private CollectionMovement collection;
    private Supplier<Double> speedSupplier;

    public MoveCollectionUp(CollectionMovement collection, Supplier<Double> speedSupplier) {
        super(collection, speedSupplier);
        this.collection = collection;
        this.speedSupplier = speedSupplier;
    }

    @Override
    public boolean isFinished() {
        return collection.canMove(speedSupplier.get());
    }
}
