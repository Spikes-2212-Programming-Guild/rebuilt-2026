package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.advancedcommands.MoveDown;
import frc.robot.commands.advancedcommands.MoveUp;
import frc.robot.commands.shooter.SimpleShoot;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;

public class TestSubsystems extends SequentialCommandGroup {

    public TestSubsystems() {
        addCommands(
                new MoveUp(),
                new MoveDown(),
                new SpinMagazine().withTimeout(2),
                new Transport().withTimeout(2),
                new SimpleShoot(() -> 0.5).withTimeout(2)
        );
    }
}