package frc.robot.commands.intake;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.intake.Collection;

import java.util.function.Supplier;

public class SpinCollection extends MoveGenericSubsystem {

    private static final double SPEED = 0.55;

    public SpinCollection(Supplier<Double> speed) {
        super(Collection.getInstance(), speed);
    }

    public SpinCollection() {
        super(Collection.getInstance(), SPEED);
    }
}
