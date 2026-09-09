package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.CollectionMovement;

public class UpWithJumpies extends SequentialCommandGroup {

    public UpWithJumpies() {
        CollectionMovement collectionMovement = CollectionMovement.getInstance();
        addCommands(
                new MiniJumpies(collectionMovement),
                new MiniJumpies(collectionMovement),
                new MoveUp()
        );
    }
}
