package frc.robot.commands.collection;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.collection.Roller;

public class SpinRoller extends MoveGenericSubsystem {

    public SpinRoller(Roller intake) {
        super(intake, Roller.SPEED);
    }
}
