package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class CollectAndPass extends ParallelCommandGroup {

    public CollectAndPass(Collection collection, CollectionMovement collectionMovement, Shooter shooter,
                          Supplier<Double> shootingSpeed, SpinningMagazine spinningMagazine, Transport transport) {
        addCommands(
                new Collect(collection, collectionMovement),
                new Pass(shooter, shootingSpeed, spinningMagazine, transport)
        );
    }
}
