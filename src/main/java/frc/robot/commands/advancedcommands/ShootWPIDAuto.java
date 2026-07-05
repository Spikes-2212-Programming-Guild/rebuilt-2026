package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.subsystems.shooter.Shooter;

public class ShootWPIDAuto extends SequentialCommandGroup {

    public ShootWPIDAuto() {
        addCommands(
                new ShootWithPID(Shooter.getInstance(), () -> 2.9, 0.25),
                new SpinMagazine().withTimeout(0.25),
                new ParallelCommandGroup (
                        new SpinMagazine(),
                        new Transport(),
                        new ShootWithPID(Shooter.getInstance(), () -> 2.9, 10)
                )
        );
    }
}
