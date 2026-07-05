package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

public class Pass extends SequentialCommandGroup {

    private static final double SHOOT_SPEED = 3.7;

    public Pass() {
        addCommands(
                new ShootWithPID(Shooter.getInstance(), () -> SHOOT_SPEED, 0.1),
                new Transport().withTimeout(0.50),
                new ParallelCommandGroup(
                        new ShootWithPID(Shooter.getInstance(), () -> SHOOT_SPEED, 20),
                        new Transport(),
                        new SpinMagazine(() -> 0.3)
                )
        );
    }
}
