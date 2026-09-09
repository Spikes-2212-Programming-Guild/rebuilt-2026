package frc.robot.commands.intake;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.intake.CollectionMovement;

import java.util.function.Supplier;

public class MoveCollection extends MoveGenericSubsystem {

    public MoveCollection(Supplier<Double> speed) {
        super(CollectionMovement.getInstance(), speed);
    }

    public MoveCollection(double speed) {
        super(CollectionMovement.getInstance(), speed);
    }
}
