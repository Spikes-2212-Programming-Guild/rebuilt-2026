package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.collection.MoveCollectionJoint;
import frc.robot.commands.collection.SpinRoller;
import frc.robot.subsystems.collection.CollectionJoint;
import frc.robot.subsystems.collection.Roller;

import java.util.function.Supplier;

public class Collect extends ParallelCommandGroup {

    private static final Supplier<Double> DOWN_SPEED = () -> -0.07;
    private static final int TIME_TO_MOVE_JOINT = 1;

    public Collect(Roller roller, CollectionJoint collectionJoint) {
        addCommands(
                new SequentialCommandGroup(
                        new MoveCollectionJoint(collectionJoint, DOWN_SPEED).withTimeout(TIME_TO_MOVE_JOINT),
                        new SequentialCommandGroup(
                                new WaitCommand(4),
                                new Jumpies(collectionJoint)
                        ).repeatedly()
                ),
                new SpinRoller(roller)
        );
    }
}
