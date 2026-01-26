package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.SpinningMagazine;

import java.util.function.Supplier;

public class SimpleTransport extends MoveGenericSubsystem {

    public SimpleTransport(SpinningMagazine spinningMagazine,
                           Supplier<Double> speed) {
        super(spinningMagazine, speed);
    }
}
