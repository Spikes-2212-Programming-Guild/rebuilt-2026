package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import subsystems.CollectionMovement;

import java.util.function.Supplier;

public class Intake extends MoveGenericSubsystem {

    private CollectionMovement collection;
    public Intake(CollectionMovement collection, Supplier<Double> speedSupplier) {
        super(collection, speedSupplier);
        this.collection = collection;
    }

    @Override
    public boolean isFinished() {
        if(collection.isAtTopLimitSwitch()){
            return true;
        } else if(collection.isAtBottomLimitSwitch()){
            return true;
        }
        return false;
    }
}
