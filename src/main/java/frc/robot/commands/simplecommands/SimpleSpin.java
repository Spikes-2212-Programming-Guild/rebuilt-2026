package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.SpinningMagazine;

public class SimpleSpin extends MoveGenericSubsystem {

    public SimpleSpin(SpinningMagazine spinningMagazine) {
        super(spinningMagazine, SpinningMagazine.SPEED);
    }
}
