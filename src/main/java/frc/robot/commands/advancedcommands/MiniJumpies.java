package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.CollectionMovement;

public class MiniJumpies extends SequentialCommandGroup {

    private static final double SPEED = 0.35;
    private static final double TIMEOUT = 0.35;

    public MiniJumpies(CollectionMovement collectionMovement) {
        addCommands(
                new SequentialCommandGroup(
                        new MoveGenericSubsystem(collectionMovement, -SPEED).withTimeout(TIMEOUT),
                        new MoveGenericSubsystem(collectionMovement, SPEED).withTimeout(TIMEOUT)
                ));
    }
}
