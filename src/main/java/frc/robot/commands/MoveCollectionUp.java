package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import subsystems.CollectionMovement;

import java.util.function.Supplier;

public class MoveCollectionUp extends MoveGenericSubsystem {

    private CollectionMovement collection;
    public MoveCollectionUp(CollectionMovement collection, Supplier<Double> speedSupplier) {
        super(collection, speedSupplier);
        this.collection = collection;
    }

    @Override
    public boolean isFinished() {
        if(collection.isAtTopLimitSwitch()){
            return true;
        }
        return false;
    }
}
