package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Collection;

public class SimpleIntake extends MoveGenericSubsystem {

    public SimpleIntake(Collection collection) {
        super(collection, Collection.SPEED);
    }
}
