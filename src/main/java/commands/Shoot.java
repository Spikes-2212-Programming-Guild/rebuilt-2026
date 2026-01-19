package commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;

import java.util.function.Supplier;

public class Shoot extends MoveGenericSubsystem {

    public Shoot(GenericSubsystem shooter, Supplier<Double> speedSupplier) {
        super(shooter, speedSupplier);
    }
}
