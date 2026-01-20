package frc.robot.commands.simplecommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Climb;

import java.util.function.Supplier;

public class SimpleClimb extends MoveGenericSubsystem {

    public SimpleClimb(Climb climb, Supplier<Double> speed) {
        super(climb, speed);
    }
}
