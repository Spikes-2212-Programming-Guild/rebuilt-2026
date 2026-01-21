package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.SpinningMagazine;
import java.util.function.Supplier;

public class Transport extends MoveGenericSubsystem {

    public Transport(SpinningMagazine spinningMagazine , Supplier<Double> speed) {
        super(spinningMagazine, speed);
    }
}
