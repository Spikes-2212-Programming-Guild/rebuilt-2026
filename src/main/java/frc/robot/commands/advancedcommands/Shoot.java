package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;

public class Shoot extends SequentialCommandGroup {

    private static final double SHOOT_SPEED = 0.3;
    private static final double KICKER_SPEED = 0.5;
    private static final double MAGAZINE_SPEED = 0.25;

    public Shoot() {
        addCommands(
                new JustShoot(Shooter.getInstance(), () -> SHOOT_SPEED).withTimeout(1),
                new Transport(Kicker.getInstance(), () -> KICKER_SPEED).withTimeout(0.25),
                new ParallelCommandGroup(
                        new JustShoot(Shooter.getInstance(), () -> SHOOT_SPEED),
                        new Transport(Kicker.getInstance(), () -> KICKER_SPEED),
                        new SpinMagazine(() -> MAGAZINE_SPEED)
                )
        );
    }
}
