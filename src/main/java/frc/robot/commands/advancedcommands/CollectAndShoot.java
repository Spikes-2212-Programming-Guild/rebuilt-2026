package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class CollectAndShoot extends ParallelCommandGroup {

    public CollectAndShoot(Collection collection, CollectionMovement collectionMovement, Shooter shooter,
                           Supplier<Double> shootingSpeed, SpinningMagazine spinningMagazine, Kicker transport) {
        addCommands(
                new Collect(collection, collectionMovement),
                new AccelerateAndShoot(shooter, shootingSpeed, spinningMagazine, transport)
        );
    }
}
