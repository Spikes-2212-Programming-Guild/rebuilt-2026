package frc.robot.commands.intake;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.intake.Collection;

import java.util.function.Supplier;

public class SpinCollection extends MoveGenericSubsystem {

    public SpinCollection(Collection collection) {
        super(collection, 0.75);
    }

    public SpinCollection(Supplier<Double> speed) {
        super(Collection.getInstance(), speed);
    }

    public SpinCollection(double speed) {
        super(Collection.getInstance(), speed);
    }
}
