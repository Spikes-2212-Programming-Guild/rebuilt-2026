package frc.robot.commands.collection;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.collection.CollectionJoint;

import java.util.function.Supplier;

public class MoveCollectionJoint extends MoveGenericSubsystem {

    public MoveCollectionJoint(CollectionJoint intakeArm, Supplier<Double> speed) {
        super(intakeArm, speed);
    }
}
