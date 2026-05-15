package frc.robot.commands.spindexer;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.SpinningMagazine;

public class SpinMagazine extends MoveGenericSubsystem {

    public SpinMagazine(SpinningMagazine spinningMagazine) {
        super(spinningMagazine, SpinningMagazine.SPEED);
    }
}
