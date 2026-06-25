package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.CollectionMovement;

public class Jumpies extends SequentialCommandGroup {

    private static final double UP_SPEED = 0.2;
    private static final double DOWN_SPEED = -0.2;

    public Jumpies(CollectionMovement collectionMovement) {
        addCommands(
                new MoveGenericSubsystem(collectionMovement, UP_SPEED).withTimeout(0.2),
                new MoveGenericSubsystem(collectionMovement, DOWN_SPEED).withTimeout(0.2)
        );
    }
}
