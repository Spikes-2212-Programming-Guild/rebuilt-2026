package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.intake.SpinCollection;

public class MoveDown extends SequentialCommandGroup {

    public MoveDown() {
        addCommands(
               new MoveCollection(0.65).withTimeout(0.30),
                new MoveCollection(0.25).withTimeout(0.27)
        );
    }
}
