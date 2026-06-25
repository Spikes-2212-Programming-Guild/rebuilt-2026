package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.intake.SpinCollection;

public class MoveDown extends SequentialCommandGroup {

    public MoveDown() {
        addCommands(
                new SpinCollection(-0.75).withTimeout(0.4),
                new MoveCollection(0.65).withTimeout(0.4),
                new MoveCollection(0.3).withTimeout(0.3)
        );
    }
}
