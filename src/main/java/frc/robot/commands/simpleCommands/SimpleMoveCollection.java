package frc.robot.commands.simpleCommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.CollectionMovement;

import java.util.function.Supplier;

public class SimpleMoveCollection extends MoveGenericSubsystem {

    public SimpleMoveCollection(CollectionMovement collection, Supplier<Double> speed) {
        super(collection, speed);
    }
}
