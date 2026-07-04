package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.advancedcommands.MoveDown;
import frc.robot.commands.advancedcommands.MoveUp;
import frc.robot.commands.shoot.SimpleShoot;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.subsystems.shooter.Shooter;

public class TestSubsystems extends SequentialCommandGroup {

    public TestSubsystems(){
        addCommands(
                new MoveUp(),
                new MoveDown(),
                new SpinMagazine().withTimeout(2),
                new Transport().withTimeout(2),
                new SimpleShoot(Shooter.getInstance(), () -> 0.5).withTimeout(2)
        );
    }
}