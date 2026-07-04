package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.shoot.SimpleShoot;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;

public class ShootAuto extends SequentialCommandGroup {

    private static final double SHOOT_SPEED = 4;

    public ShootAuto() {
        addCommands(
                new ShootWithPID(() -> SHOOT_SPEED, 1),
                new Transport().withTimeout(0.25),
                new ParallelCommandGroup(
                        new ShootWithPID(() -> SHOOT_SPEED, 10),
                        new SimpleShoot(() -> SHOOT_SPEED),
                        new Transport(),
                        new SpinMagazine()
                )
        );
    }
}
