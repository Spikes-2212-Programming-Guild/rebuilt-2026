package frc.robot.commands.intake;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.forbar.Collection;

public class Intake extends MoveGenericSubsystem {

    public Intake(Collection collection) {
        super(collection, Collection.SPEED);
    }
}
