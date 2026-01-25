package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Collection;

import java.util.function.Supplier;

public class SimpleIntake extends MoveGenericSubsystem {

    public SimpleIntake(Collection collection, Supplier<Double> speed) {
        super(collection, speed);
    }

}
