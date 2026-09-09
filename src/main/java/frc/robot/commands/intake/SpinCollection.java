package frc.robot.commands.intake;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Collection;

import java.util.function.Supplier;

public class SpinCollection extends MoveGenericSubsystem {

    private static final Supplier<Double> SPEED = Robot.namespace.addConstantDouble("spin speed", 0.75);

    public SpinCollection(Supplier<Double> speed) {
        super(Collection.getInstance(), speed);
    }

    public SpinCollection() {
        super(Collection.getInstance(), SPEED);
    }
}
