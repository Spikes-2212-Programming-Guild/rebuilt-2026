package frc.robot.commands.simple.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Collection;

import java.util.function.Supplier;

public class Intake extends MoveGenericSubsystem {

    public Intake(Collection collection, Supplier<Double> speed) {
        super(collection, speed);
        collection.configureDashboard();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
