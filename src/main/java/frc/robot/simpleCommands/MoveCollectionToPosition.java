package frc.robot.simpleCommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class MoveCollectionToPosition extends MoveGenericSubsystem {

    public MoveCollectionToPosition(CollectionMovement collection, Supplier<Double> speed) {
        super(collection, speed);
    }
}
