package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.commands.simplecommands.SimpleShoot;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class ShootButSlowly extends ParallelCommandGroup {

    private static final double SHOOTING_START_SPEED = 0.2;
    private static final double TIMEOUT = 0.25;

    public ShootButSlowly(Shooter shooter, Supplier<Double> shooterSpeed) {
        addCommands(
                new SequentialCommandGroup(
                        new SimpleShoot(shooter, ()-> SHOOTING_START_SPEED).withTimeout(TIMEOUT),
                        new ShootWithPID(shooter, shooterSpeed)
                )
        );
    }
}
