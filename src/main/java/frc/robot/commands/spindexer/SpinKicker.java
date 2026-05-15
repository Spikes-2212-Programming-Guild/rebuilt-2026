package frc.robot.commands.spindexer;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.Kicker;

public class SpinKicker extends MoveGenericSubsystem {

    public SpinKicker(Kicker kicker) {
        super(kicker, Kicker.SPEED);
    }
}
