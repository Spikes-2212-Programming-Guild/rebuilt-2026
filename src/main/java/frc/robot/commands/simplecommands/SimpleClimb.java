package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;

import java.util.function.Supplier;

public class SimpleClimb extends MoveGenericSubsystem {

    public SimpleClimb(GenericSubsystem subsystem, Supplier<Double> speedSupplier) {
        super(subsystem, speedSupplier);
    }
}
