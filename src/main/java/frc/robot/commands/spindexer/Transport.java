package frc.robot.commands.spindexer;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.Kicker;

import java.util.function.Supplier;

public class Transport extends MoveGenericSubsystem {

    private static final double SPEED = 0.5;

    public Transport(Supplier<Double> speed) {
        super(Kicker.getInstance(), speed);
    }

    public Transport() {
        super(Kicker.getInstance(), SPEED);
    }
}
