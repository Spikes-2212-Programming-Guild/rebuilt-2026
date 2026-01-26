package frc.robot.commands.simpleCommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Transport;

public class SimpleTransport extends MoveGenericSubsystem {

    public SimpleTransport(Transport transport) {
        super(transport, Transport.SPEED);
    }
}
