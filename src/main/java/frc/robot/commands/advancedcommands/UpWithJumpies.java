package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.CollectionMovement;

public class UpWithJumpies extends SequentialCommandGroup {

    public UpWithJumpies(CollectionMovement collectionMovement) {
        addCommands(
                new MiniJumpies(collectionMovement),
                new MiniJumpies(collectionMovement),
                new MiniJumpies(collectionMovement),
                new MiniJumpies(collectionMovement),
                new MiniJumpies(collectionMovement),
                new MoveUp()
        );
    }
}
