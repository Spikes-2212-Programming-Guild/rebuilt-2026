package frc.robot.commands.storage;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.Kicker;

public class Transport extends MoveGenericSubsystem {

    public Transport(Kicker transport) {
        super(transport, Kicker.SPEED);
    }
}
