package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.intake.SpinCollection;

public class MoveUp extends SequentialCommandGroup {

    public MoveUp() {
        addCommands(
                new ParallelCommandGroup(
                        new SpinCollection(),
                        new MoveCollection(-0.55)
                ).withTimeout(0.6),
                new MoveCollection(-0.3).withTimeout(0.25)
        );
    }
}
