package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;

public class MoveUpRegular extends SequentialCommandGroup {

    public MoveUpRegular(){
        addCommands(
                new MoveCollection(-0.55).withTimeout(0.4),
                new MoveCollection(-0.3).withTimeout(0.25)
        );
    }
}
