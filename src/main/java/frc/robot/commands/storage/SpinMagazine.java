package frc.robot.commands.storage;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class SpinMagazine extends MoveGenericSubsystem {

    private static final double SPEED = 0.1;

    public SpinMagazine(Supplier<Double> speed) {
        super(SpinningMagazine.getInstance(), speed);
    }

    public SpinMagazine() {
        super(SpinningMagazine.getInstance(), SPEED);
    }
}
