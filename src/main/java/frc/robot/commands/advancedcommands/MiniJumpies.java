package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.CollectionMovement;

public class MiniJumpies extends SequentialCommandGroup {

    private static final double UP_SPEED = 0.45;
    private static final double DOWN_SPEED = -0.45;

    public MiniJumpies(CollectionMovement collectionMovement) {
        addCommands(
                new SequentialCommandGroup(
                new MoveGenericSubsystem(collectionMovement, UP_SPEED).withTimeout(0.35),
                new MoveGenericSubsystem(collectionMovement, DOWN_SPEED).withTimeout(0.35)
        ));
    }
}
