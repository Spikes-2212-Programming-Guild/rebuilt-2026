package frc.robot.commands.storage;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;

public class Transport extends MoveGenericSubsystem {

    public Transport(frc.robot.subsystems.spindexer.Transport transport) {
        super(transport, frc.robot.subsystems.spindexer.Transport.SPEED);
    }
}
