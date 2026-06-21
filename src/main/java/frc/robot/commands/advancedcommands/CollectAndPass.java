package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.intake.CollectionMovement;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class CollectAndPass extends ParallelCommandGroup {

    public CollectAndPass(Collection collection, CollectionMovement collectionMovement, Shooter shooter,
                          Supplier<Double> shootingSpeed, SpinningMagazine spinningMagazine, Kicker transport) {
        addCommands(
                new Collect(collection, collectionMovement),
                new Pass(shooter, shootingSpeed, spinningMagazine, transport)
        );
    }
}
