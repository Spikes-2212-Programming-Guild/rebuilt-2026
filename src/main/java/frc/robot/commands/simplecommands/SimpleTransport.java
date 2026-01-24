package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Transport;

import java.util.function.Supplier;

public class SimpleTransport extends MoveGenericSubsystem {

    private final Transport transport;

    public SimpleTransport(Transport transport, Supplier<Double> speed) {
        super(transport, speed);
        this.transport = transport;
    }
}