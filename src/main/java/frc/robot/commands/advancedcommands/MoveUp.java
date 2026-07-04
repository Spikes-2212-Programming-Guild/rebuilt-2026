package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.intake.SpinCollection;

public class MoveUp extends SequentialCommandGroup {

    public MoveUp() {
        addCommands(
//                new SpinCollection(-0.75).withTimeout(0.5),
                new MoveCollection(-0.55).withTimeout(0.4),
                new MoveCollection(-0.3).withTimeout(0.25)
        );
    }
}
