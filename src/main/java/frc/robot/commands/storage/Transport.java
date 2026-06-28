package frc.robot.commands.storage;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.Kicker;

import java.util.function.Supplier;

public class Transport extends MoveGenericSubsystem {

    public Transport(Kicker kicker) {
        super(kicker, Kicker.SPEED);
    }

    public Transport(Kicker kicker, Supplier<Double> speed) {
        super(kicker, speed);
    }


    public Transport() {
        super(Kicker.getInstance(), () -> 0.5);
    }
}
