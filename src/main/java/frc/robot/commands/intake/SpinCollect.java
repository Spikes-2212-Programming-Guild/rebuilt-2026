package frc.robot.commands.intake;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.intake.Collection;

public class SpinCollect extends MoveGenericSubsystem {

    public SpinCollect(Collection collection) {
        super(collection, Collection.SPEED);
    }
}
