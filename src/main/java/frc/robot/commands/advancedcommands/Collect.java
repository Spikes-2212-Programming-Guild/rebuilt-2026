package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.SpinCollection;

public class Collect extends SequentialCommandGroup {

    public Collect() {
        addCommands(
                new MoveDown(),
                new SpinCollection(0.75)
        );
    }
}
