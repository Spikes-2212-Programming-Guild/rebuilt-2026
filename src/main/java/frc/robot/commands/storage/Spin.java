package frc.robot.commands.storage;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.spindexer.SpinningMagazine;

public class Spin extends MoveGenericSubsystem {

    public Spin(SpinningMagazine spinningMagazine) {
        super(spinningMagazine, SpinningMagazine.SPEED);
    }
}
