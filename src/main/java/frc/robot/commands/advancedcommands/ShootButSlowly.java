package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class ShootButSlowly extends ParallelCommandGroup {

    private static final double SHOOTING_START_SPEED = -1.0;

    public ShootButSlowly(Shooter shooter, Supplier<Double> shooterSpeed) {
        addCommands(
                new SequentialCommandGroup(
                        new ShootWithPID(shooter, ()-> SHOOTING_START_SPEED),
                        new ShootWithPID(shooter, shooterSpeed)
                )
        );
    }
}
